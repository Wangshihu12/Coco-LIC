/*
 * Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <pcl/kdtree/impl/kdtree_flann.hpp> // pcl::KdTreeFLANN

#include <lidar/lidar_handler.h>
// #include <lidar/livox_feature_extraction.h>
#include <lidar/velodyne_feature_extraction.h>
#include <pcl/common/transforms.h>           //pcl::transformPointCloud
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg

#include <algorithm> // std::find

namespace cocolic
{

  struct by_timestamp
  {
    bool operator()(const PointCorrespondence &left,
                    const PointCorrespondence &right)
    {
      return left.t_point < right.t_point;
    }
  };

  LidarHandler::LidarHandler(const YAML::Node &node, Trajectory::Ptr traj)
      : trajectory_(std::move(traj)),
        use_corner_feature_(true),
        cor_downsample_(1),
        update_full_cloud_(false),
        kdtree_corner_map_(new pcl::KdTreeFLANN<PosPoint>()),
        kdtree_surface_map_(new pcl::KdTreeFLANN<PosPoint>()),
        latest_feature_time_(-1),
        cloud_key_pos_(new PosCloud),
        cloud_key_pos_xy_(new PosCloud),
        key_frame_updated_(false)
  {
    use_corner_feature_ = yaml::GetBool(node, "use_corner_feature", false);

    const YAML::Node &map_node = node["map_param"];
    keyframe_search_radius_ = map_node["keyframe_search_radius"].as<float>();
    keyframe_search_time_ = map_node["keyframe_search_time"].as<float>() * S_TO_NS;
    keyframe_density_map_ = map_node["keyframe_density"].as<float>();

    cloud_reserved_time_ =
        yaml::GetValue<double>(map_node, "cloud_reserved_time", -1) * S_TO_NS;

    const YAML::Node &cur_scan_node = node["current_scan_param"];
    edge_min_valid_num_ = cur_scan_node["edge_min_valid_num"].as<int>();
    surf_min_valid_num_ = cur_scan_node["surf_min_valid_num"].as<int>();
    corner_leaf_size_ = cur_scan_node["corner_leaf_size"].as<float>();
    surface_leaf_size_ = cur_scan_node["surface_leaf_size"].as<float>();
    cor_downsample_ = cur_scan_node["correspondence_downsample"].as<int>();

    const YAML::Node &keyframe_node = node["keyframe_strategy"];
    keyframe_angle_degree_ = keyframe_node["angle_degree"].as<double>();
    keyframe_dist_meter_ = keyframe_node["dist_meter"].as<double>();
    keyframe_time_second_ = keyframe_node["time_second"].as<double>() * S_TO_NS;

    feature_cur_vec_.clear();
  }

  // Note 
  void LidarHandler::FeatureCloudHandler(
      const cocolic::feature_cloud::ConstPtr &feature_msg)
  {
    RTPointCloud::Ptr corner_cloud(new RTPointCloud);
    RTPointCloud::Ptr surface_cloud(new RTPointCloud);
    RTPointCloud::Ptr full_cloud(new RTPointCloud);

    pcl::fromROSMsg(feature_msg->corner_cloud, *corner_cloud);
    pcl::fromROSMsg(feature_msg->surface_cloud, *surface_cloud);
    pcl::fromROSMsg(feature_msg->full_cloud, *full_cloud);

    feature_cur_.Clear();

    double scan_time = feature_msg->header.stamp.toSec();
    double traj_start_time = trajectory_->GetDataStartTime();
    feature_cur_.timestamp = scan_time - traj_start_time;
    CloudToRelativeMeasureTime(corner_cloud, scan_time, traj_start_time);
    CloudToRelativeMeasureTime(surface_cloud, scan_time, traj_start_time);
    CloudToRelativeMeasureTime(full_cloud, scan_time, traj_start_time);

    if (use_corner_feature_)
    {
      pcl::RTPointCloudToPosCloud(corner_cloud, feature_cur_.corner_features);
    }
    pcl::RTPointCloudToPosCloud(surface_cloud, feature_cur_.surface_features);
    pcl::RTPointCloudToPosCloud(full_cloud, feature_cur_.full_cloud,
                                &feature_cur_.time_max);
    /// 
    DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
  }

  // Note 
  void LidarHandler::FeatureCloudHandler(const LiDARFeature &lidar_feature)
  {
    feature_cur_.Clear();
    feature_cur_.time_max = lidar_feature.time_max;
    feature_cur_.timestamp = lidar_feature.timestamp;
    *(feature_cur_.corner_features) = *(lidar_feature.corner_features);
    *(feature_cur_.surface_features) = *(lidar_feature.surface_features);
    *(feature_cur_.full_cloud) = *(lidar_feature.full_cloud);

    /// 
    DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
  }

  // Note 
  void LidarHandler::FeatureCloudHandler(int64_t scan_timestamp,
                                          int64_t scan_time_max,
                                          const RTPointCloud::Ptr corner_feature,
                                          const RTPointCloud::Ptr surface_feature,
                                          const RTPointCloud::Ptr raw_cloud)
  {
    feature_cur_.Clear();
    feature_cur_.timestamp = scan_timestamp;
    feature_cur_.time_max = scan_time_max;
    if (use_corner_feature_)
    {
      pcl::RTPointCloudToPosCloud(corner_feature, feature_cur_.corner_features);
    }
    pcl::RTPointCloudToPosCloud(surface_feature, feature_cur_.surface_features);
    pcl::RTPointCloudToPosCloud(raw_cloud, feature_cur_.full_cloud);

    // feature_cur_vec_.push_back(feature_cur_);

    // downsample the current lidar feature
    DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
  }

  // 更新激光雷达子地图：基于关键帧管理策略更新局部地图
  bool LidarHandler::UpdateLidarSubMap()
  {
    // 更新关键帧：检查是否需要添加新的关键帧
    UpdateKeyFrames();
    
    // 如果关键帧有更新
    if (key_frame_updated_)
    {
      key_frame_updated_ = false;  // 重置关键帧更新标志
      
      // 提取当前时刻周围的特征点构建子地图
      ExtractSurroundFeatures(feature_cur_.timestamp);
      
      return true;   // 返回true表示子地图已更新
    }
    else
    {
      return false;  // 返回false表示子地图未更新
    }
  }

  // 获取LOAM特征关联：建立当前帧特征与地图特征之间的对应关系
  void LidarHandler::GetLoamFeatureAssociation()
  {
    // 创建存储地图坐标系下当前激光雷达特征的容器
    LiDARFeature cur_lf_in_map;

    // 如果启用角点特征
    if (use_corner_feature_)
    {
      // 将当前帧的角点特征从激光雷达坐标系去畸变到全局地图坐标系
      trajectory_->UndistortScanInG(*feature_cur_ds_.corner_features,    // 输入：当前帧下采样后的角点特征
                                    feature_cur_ds_.timestamp,           // 特征的时间戳
                                    *cur_lf_in_map.corner_features);     // 输出：地图坐标系下的角点特征
    }
    
    // 将当前帧的表面特征从激光雷达坐标系去畸变到全局地图坐标系
    trajectory_->UndistortScanInG(*feature_cur_ds_.surface_features,     // 输入：当前帧下采样后的表面特征
                                  feature_cur_ds_.timestamp,             // 特征的时间戳
                                  *cur_lf_in_map.surface_features);      // 输出：地图坐标系下的表面特征

    // 寻找特征对应关系：在当前帧特征和地图特征之间建立最近邻对应关系
    FindCorrespondence(feature_cur_ds_,      // 当前帧特征（激光雷达坐标系）
                      cur_lf_in_map);       // 当前帧特征（地图坐标系）

    // 按时间戳对点对应关系进行排序
    // 这样可以确保优化时按时间顺序处理特征匹配
    std::sort(point_correspondence_.begin(), point_correspondence_.end(),
              by_timestamp());

    // 对特征对应关系进行下采样
    // 减少计算量，提高优化效率，同时保持足够的约束
    DownSampleCorrespondence();

    // 输出调试信息：显示原始和下采样后的对应关系数量
    // LOG(INFO) << "point correspondence : " << point_correspondence_.size()
    //           << "; point correspondence_ds : "
    //           << point_correspondence_ds_.size();
  }

  bool LidarHandler::UpdateKeyFrames()
  {
    // 
    LiDARFeature feature;
    feature = feature_cur_;
    cache_feature_container_[feature.timestamp] = feature;

    // 
    for (auto iter = cache_feature_container_.begin();
         iter != cache_feature_container_.end();)
    {
      /// [Case 1] 
      if (!IsKeyFrame(iter->second.time_max))
      {
        // 
        if (iter->second.time_max >=
            trajectory_->GetActiveTime())
        { // 
          break;
        }
        else
        {
          cache_feature_container_.erase(iter++); // 
          continue;
        }
      }

      /// [Case 2] 
      LiDARFeature key_scan;
      key_scan.timestamp = iter->second.timestamp;
      key_scan.time_max = iter->second.time_max;
      if (local_feature_container_.empty())
      {
        // 
        // key_pose.timestamp = trajectory_->minTime(LiDARSensor);
        *(key_scan.corner_features) = *(iter->second.corner_features);
        *(key_scan.surface_features) = *(iter->second.surface_features);
        *(key_scan.full_cloud) = *(iter->second.full_cloud);
      }
      else
      {
        // 
        trajectory_->UndistortScan(*(iter->second.corner_features),
                                   key_scan.timestamp,
                                   *(key_scan.corner_features));
        trajectory_->UndistortScan(*(iter->second.surface_features),
                                   key_scan.timestamp,
                                   *(key_scan.surface_features));
        trajectory_->UndistortScan(*(iter->second.full_cloud), key_scan.timestamp,
                                   *(key_scan.full_cloud));
      }
#if false
    if (local_feature_container_.size() < 10)
      local_feature_container_[key_scan.timestamp] = key_scan;
    else {
      LiDARFeature key_scan_ds;
      DownsampleLiDARFeature(key_scan, key_scan_ds);
      local_feature_container_[key_scan.timestamp] = key_scan_ds;
    }
#else
      LiDARFeature key_scan_ds;
      DownsampleLiDARFeature(key_scan, key_scan_ds);
      local_feature_container_[key_scan.timestamp] = key_scan;
      local_feature_container_all_ds_[key_scan.timestamp] = key_scan_ds;
      // 
      latest_feature_time_ = key_scan.timestamp;
#endif
      cache_feature_container_.erase(iter++);

      PosPoint key_pose;
      key_pose.timestamp = key_scan.timestamp;
      Eigen::Vector3d p =
          trajectory_->GetLidarPoseNURBS(key_scan.timestamp).translation();
      key_pose.x = p[0];
      key_pose.y = p[1];
      key_pose.z = p[2];
      cloud_key_pos_->push_back(key_pose);

      key_pose.z = 0;
      cloud_key_pos_xy_->push_back(key_pose);

      // trajectory_->SetForcedFixedTime(key_scan.time_max);

      key_frame_updated_ = true;
    }

    // 
    if (cloud_reserved_time_ > 0)
    {
      double t_now = local_feature_container_.rbegin()->first;
      auto iter = local_feature_container_.begin();
      while (iter != local_feature_container_.end())
      {
        if (t_now - (*iter).first > cloud_reserved_time_)
        {
          iter = local_feature_container_.erase(iter);
        }
        else
        {
          break;
        }
      }
    }

    return key_frame_updated_;
  }

  const bool LidarHandler::IsKeyFrame(int64_t time) const
  {
    // 
    if (cloud_key_pos_->points.empty())
    {
      // LOG(INFO) << "[first_lidar time_max] " << time * NS_TO_S;
      return true;
    }

    // 
    // 
    if (time >= trajectory_->GetActiveTime())
      return false;

    // 
    if ((time - cloud_key_pos_->back().timestamp) > keyframe_time_second_)
      return true;

    SE3d pose_cur = trajectory_->GetLidarPoseNURBS(time);
    SE3d pose_last = trajectory_->GetLidarPoseNURBS(cloud_key_pos_->back().timestamp);

    SE3d pose_cur_to_last = pose_last.inverse() * pose_cur;
    Eigen::AngleAxisd v(pose_cur_to_last.so3().unit_quaternion());

    double dist_meter = pose_cur_to_last.translation().norm();
    double angle_degree = v.angle() * (180. / M_PI);

    if (angle_degree > keyframe_angle_degree_ ||
        dist_meter > keyframe_dist_meter_)
    {
      //    LOG(INFO) << WHITE << "[ IsKeyFrame ] " << angle_degree << " : "
      //              << dist_meter;
      return true;
    }

    return false;
  }

  void LidarHandler::GetNearTimeKeyScanID(PosCloud::Ptr key_pos_selected,
                                           const int64_t cur_time) const
  {
    std::vector<double> t_added;
    t_added.reserve(key_pos_selected->size());
    for (size_t i = 0; i < key_pos_selected->size(); i++)
    {
      t_added.push_back(key_pos_selected->points[i].timestamp);
    }

    /// 
    int64_t map_start_time = cur_time - keyframe_search_time_;
    // float map_start_time = cloud_key_pos_->points.back().timestamp - keyframe_search_time_;
    for (int i = cloud_key_pos_->size() - 1; i >= 0; i--)
    {
      if (cloud_key_pos_->points[i].timestamp < map_start_time)
        break;

      /// 
      auto iter = std::find(t_added.begin(), t_added.end(),
                            cloud_key_pos_->points[i].timestamp);
      if (t_added.end() == iter)
      {
        key_pos_selected->push_back(cloud_key_pos_->points[i]);
      }
    }
  }

  void LidarHandler::GetNearDistKeyScanID(PosCloud::Ptr key_pos_selected)
  {
    PosCloud::Ptr nearby_key_pos(new PosCloud);

    std::vector<int> indices;
    std::vector<float> sqr_dists;
    pcl::KdTreeFLANN<PosPoint>::Ptr kdtree_nearby_key_pos(
        new pcl::KdTreeFLANN<PosPoint>());
    kdtree_nearby_key_pos->setInputCloud(cloud_key_pos_);
    kdtree_nearby_key_pos->radiusSearch(cloud_key_pos_->back(),
                                        (double)keyframe_search_radius_, indices,
                                        sqr_dists);
    for (const int &idx : indices)
    {
      nearby_key_pos->push_back(cloud_key_pos_->points[idx]);
    }
    if (cloud_key_pos_->size() > 20)
    {
      VoxelFilter<PosPoint> surrounding_key_pos_voxel_filter;
      surrounding_key_pos_voxel_filter.SetResolution(keyframe_density_map_);
      surrounding_key_pos_voxel_filter.SetInputCloud(nearby_key_pos);
      surrounding_key_pos_voxel_filter.Filter(key_pos_selected);
    }
    else
    {
      key_pos_selected = nearby_key_pos;
    }
  }

  void LidarHandler::TransformLiDARFeature(const LiDARFeature &lf_in,
                                            const Eigen::Matrix4d &T_in_to_out,
                                            LiDARFeature &lf_out) const
  {
    if (use_corner_feature_)
    {
      pcl::transformPointCloud(*lf_in.corner_features, *lf_out.corner_features,
                               T_in_to_out);
    }
    pcl::transformPointCloud(*lf_in.surface_features, *lf_out.surface_features,
                             T_in_to_out);

    if (update_full_cloud_)
    {
      pcl::transformPointCloud(*lf_in.full_cloud, *lf_out.full_cloud,
                               T_in_to_out);
    }

    lf_out.timestamp = lf_in.timestamp;
    lf_out.time_max = lf_in.time_max;
  }

  void LidarHandler::DownsampleLiDARFeature(const LiDARFeature &lf_in,
                                             LiDARFeature &lf_out) const
  {
    lf_out.Clear();
    lf_out.timestamp = lf_in.timestamp;
    lf_out.time_max = lf_in.time_max;

    if (use_corner_feature_)
    {
      VoxelFilter<PosPoint> corner_features_voxel_filter;
      corner_features_voxel_filter.SetResolution(corner_leaf_size_);
      corner_features_voxel_filter.SetInputCloud(lf_in.corner_features);
      corner_features_voxel_filter.Filter(lf_out.corner_features);
    }

    VoxelFilter<PosPoint> surface_features_voxel_filter;
    surface_features_voxel_filter.SetResolution(surface_leaf_size_);
    surface_features_voxel_filter.SetInputCloud(lf_in.surface_features);
    surface_features_voxel_filter.Filter(lf_out.surface_features);
  }

  // 提取周围特征：从关键帧中构建当前时刻的局部特征地图
  void LidarHandler::ExtractSurroundFeatures(const int64_t cur_time)
  {
    // 断言检查：确保存在关键帧数据来构建局部地图
    assert(!cloud_key_pos_->points.empty() && "no key scan to build local map");

    /// 选择合适的关键帧来构建局部地图
    PosCloud::Ptr key_pos_selected(new PosCloud);  // 选中的关键帧位置
    GetNearDistKeyScanID(key_pos_selected);         // 根据距离选择附近的关键帧
    GetNearTimeKeyScanID(key_pos_selected, cur_time); // 根据时间选择附近的关键帧
    // LOG(INFO) << "[keysize] " << key_pos_selected->points.size();

    // 获取目标时间：使用最新关键帧的时间戳
    int64_t target_time = cloud_key_pos_->back().timestamp;
    
    // 清空并初始化特征地图
    feature_map_.Clear();
    feature_map_.timestamp = target_time;

    // 如果只有一个关键帧的情况
    if (local_feature_container_.size() == 1)
    {
      int64_t kf_time = target_time;  // 关键帧时间

      // 获取当前激光雷达到全局坐标系的变换矩阵 TG_Lcur
      SE3d pose_cur_to_G = trajectory_->GetLidarPoseNURBS(kf_time);
      LiDARFeature lf_out;  // 输出的激光雷达特征
      
      // 将关键帧特征从激光雷达坐标系变换到全局坐标系
      TransformLiDARFeature(local_feature_container_[kf_time],
                            pose_cur_to_G.matrix(), lf_out);

      // 添加角点特征（如果启用）
      if (use_corner_feature_)
      {
        *feature_map_.corner_features += *(lf_out.corner_features);
      }
      // 添加表面特征
      *feature_map_.surface_features += *(lf_out.surface_features);
      // 添加完整点云（如果需要更新）
      if (update_full_cloud_)
      {
        *feature_map_.full_cloud += *(lf_out.full_cloud);
      }
    }
    else  // 如果有多个关键帧
    {
      // 遍历所有选中的关键帧位置
      for (auto const &pos : key_pos_selected->points)
      {
        int64_t kf_time = pos.timestamp;  // 关键帧时间戳

        // 检查该关键帧是否在本地特征容器中
        if (local_feature_container_.find(kf_time) !=
            local_feature_container_.end())
        {
          // 获取该关键帧到全局坐标系的变换矩阵 TG_Lcur
          SE3d pose_cur_to_G = trajectory_->GetLidarPoseNURBS(kf_time);
          LiDARFeature lf_out;  // 输出的激光雷达特征
          
          // 将关键帧特征变换到全局坐标系
          TransformLiDARFeature(local_feature_container_[kf_time],
                                pose_cur_to_G.matrix(), lf_out);

          // 累加角点特征（如果启用）
          if (use_corner_feature_)
          {
            *feature_map_.corner_features += *lf_out.corner_features;
          }
          // 累加表面特征
          *feature_map_.surface_features += *lf_out.surface_features;
          // 累加完整点云（如果需要更新）
          if (update_full_cloud_)
          {
            *feature_map_.full_cloud += *lf_out.full_cloud;
          }

          // 更新特征地图的最大时间
          feature_map_.time_max =
              std::max(feature_map_.time_max, lf_out.time_max);
        }
      }
    }

    // 根据点云密度决定是否需要下采样
    if (feature_map_.surface_features->size() < 10000)  // 如果点数较少
    {
      // 直接复制，不进行下采样
      feature_map_ds_.Clear();
      feature_map_ds_ = feature_map_;
    }
    else  // 如果点数较多
    {
      // 对特征地图进行下采样以提高计算效率
      DownsampleLiDARFeature(feature_map_, feature_map_ds_);
    }

    // 输出调试信息
    // LOG(INFO) << "[feature_map_ds] "
    //           << "surface cloud map : "
    //           << feature_map_ds_.surface_features->size()
    //           << "; cornor cloud map: " << feature_map_ds_.corner_features->size()
    //           << "; feature map time : [" << feature_map_ds_.timestamp << ", "
    //           << feature_map_ds_.time_max << "].";

    // 设置目标地图用于后续的点云配准
    SetTargetMap(feature_map_ds_);
  }

  void LidarHandler::SetTargetMap(const LiDARFeature &feature_map)
  {
    assert(!feature_map.surface_features->empty() &&
           "[SetTargetMap] input surface_features are empty");
    assert(!(use_corner_feature_ && feature_map.corner_features->empty()) &&
           "[SetTargetMap] input corner_features are empty");

    if (use_corner_feature_)
      kdtree_corner_map_->setInputCloud(feature_map.corner_features);
    kdtree_surface_map_->setInputCloud(feature_map.surface_features);
  }

  // 寻找特征对应关系：在当前帧特征和地图特征之间建立几何对应关系
  bool LidarHandler::FindCorrespondence(const LiDARFeature &lf_cur,        // 当前帧特征（激光雷达坐标系）
                                        const LiDARFeature &lf_cur_in_M)   // 当前帧特征（地图坐标系）
  {
    // 清空并预分配点对应关系容器
    point_correspondence_.clear();
    point_correspondence_.reserve(lf_cur.surface_features->size()); // 预分配内存，大小基于表面特征数量

    // 检查特征点数量是否满足最低要求
    if ((use_corner_feature_ &&
        lf_cur.corner_features->size() < size_t(edge_min_valid_num_)) &&
        lf_cur.surface_features->size() < size_t(surf_min_valid_num_))
    {
      // 如果角点和表面特征都不足，无法进行有效匹配
      // LOG(WARNING) << "[FindCorrespondence] No enough feature points ! Corner: "
      //              << lf_cur.corner_features->size()
      //              << "; Surface: " << lf_cur.surface_features->size();
      return false;
    }

    /// 角点特征匹配：建立点到线的几何约束
    size_t corner_step = lf_cur.corner_features->size() / 1500 + 1;  // 计算角点采样步长，最多处理1500个点
    // LOG(INFO) << "[corner_step] " << corner_step;
    
    if (use_corner_feature_ && feature_map_ds_.corner_features->size() > 10)
    {
      // 遍历当前帧的角点特征（按步长采样）
      for (size_t i = 0; i < lf_cur.corner_features->size(); i += corner_step)
      {
        // 获取地图坐标系下的当前点
        PosPoint point_inM = lf_cur_in_M.corner_features->points[i];
        std::vector<int> k_indices;      // KD树搜索结果：邻居点索引
        std::vector<float> k_sqr_dists;  // KD树搜索结果：邻居点距离平方
        
        // 在地图的角点特征中搜索5个最近邻
        kdtree_corner_map_->nearestKSearch(point_inM, 5, k_indices, k_sqr_dists);

        // 如果第5个最近邻距离过大，说明匹配质量不好，跳过
        if (k_sqr_dists[4] > 1.0)
          continue;

        // 准备主成分分析(PCA)的矩阵
        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));  // 协方差矩阵
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));  // 特征值
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));  // 特征向量

        // 计算5个最近邻点的中心点
        Eigen::Vector3d cp(0, 0, 0);
        for (int j = 0; j < 5; j++)
        {
          cp[0] += feature_map_ds_.corner_features->points[k_indices[j]].x;
          cp[1] += feature_map_ds_.corner_features->points[k_indices[j]].y;
          cp[2] += feature_map_ds_.corner_features->points[k_indices[j]].z;
        }
        cp /= 5.0;

        // 计算协方差矩阵的元素
        float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
        for (int j = 0; j < 5; j++)
        {
          // 计算每个点相对于中心点的偏移
          float ax = feature_map_ds_.corner_features->points[k_indices[j]].x - cp[0];
          float ay = feature_map_ds_.corner_features->points[k_indices[j]].y - cp[1];
          float az = feature_map_ds_.corner_features->points[k_indices[j]].z - cp[2];

          // 累加协方差矩阵元素
          a11 += ax * ax;  a12 += ax * ay;  a13 += ax * az;
          a22 += ay * ay;  a23 += ay * az;  a33 += az * az;
        }
        // 计算平均值（协方差）
        a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

        // 构建协方差矩阵
        matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

        /// 计算特征值和特征向量
        cv::eigen(matA1, matD1, matV1);

        /// 检查是否形成线特征：最大特征值应该远大于其他特征值
        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
        {
  #if 1
          // 当前点坐标
          float x0 = point_inM.x;
          float y0 = point_inM.y;
          float z0 = point_inM.z;
          
          // 沿主方向构造线段的两个端点
          float x1 = cp[0] + 0.1 * matV1.at<float>(0, 0);
          float y1 = cp[1] + 0.1 * matV1.at<float>(0, 1);
          float z1 = cp[2] + 0.1 * matV1.at<float>(0, 2);
          float x2 = cp[0] - 0.1 * matV1.at<float>(0, 0);
          float y2 = cp[1] - 0.1 * matV1.at<float>(0, 1);
          float z2 = cp[2] - 0.1 * matV1.at<float>(0, 2);

          // 计算点到线的距离（使用叉积公式）
          float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          // 线段长度
          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

          // 计算雅可比矩阵的系数（用于优化）
          float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;
          float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
          float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

          // 点到线的距离
          float ld2 = a012 / l12;

          // 计算权重：距离越近权重越大
          float s = 1 - 0.9 * fabs(ld2);

          // 如果权重过小，说明匹配质量不好，跳过
          if (s <= 0.1)
          {
            std::cout << YELLOW << "bad corner association\n" << RESET;
            continue;
          }
  #endif

          // 构造点对应关系
          Eigen::Vector3d normal(matV1.at<float>(0, 0), matV1.at<float>(0, 1), matV1.at<float>(0, 2));
          
          PointCorrespondence point_cor;
          point_cor.geo_type = Line;  // 几何类型：线
          point_cor.t_point = lf_cur.corner_features->points[i].timestamp;  // 点的时间戳
          point_cor.t_map = feature_map_.timestamp;                         // 地图时间戳
          point_cor.geo_point = cp;                                         // 线上的参考点
          point_cor.geo_normal = normal;                                    // 线的方向向量
          point_cor.point = Eigen::Vector3d(lf_cur.corner_features->points[i].x,
                                            lf_cur.corner_features->points[i].y,
                                            lf_cur.corner_features->points[i].z);  // 当前点坐标
          point_cor.scale = s;  // 匹配权重
          point_correspondence_.push_back(point_cor);
        }
      }
    }
    int corner_feature_num = point_correspondence_.size();  // 记录角点匹配数量

    /// 表面特征匹配：建立点到面的几何约束
    map_corrs_viewer.clear();  // 清空可视化容器
    size_t surf_step = lf_cur.surface_features->size() / 1500 + 1;  // 计算表面点采样步长
    // LOG(INFO) << "[surf_step] " << surf_step;
    
    for (size_t i = 0; i < lf_cur.surface_features->size(); i += surf_step)
    {
      // 跳过无效时间戳的点
      if (lf_cur.surface_features->points[i].timestamp < 0)
        continue;

      // 获取地图坐标系下的当前点
      PosPoint point_inM = lf_cur_in_M.surface_features->points[i];
      std::vector<int> k_indices;
      std::vector<float> k_sqr_dists;
      
      // 在地图的表面特征中搜索5个最近邻
      kdtree_surface_map_->nearestKSearch(point_inM, 5, k_indices, k_sqr_dists);

      // 如果第5个最近邻距离过大，跳过
      if (k_sqr_dists[4] > 1.0)
        continue;

      // 准备平面拟合的矩阵
      Eigen::Matrix<float, 5, 3> matA0;  // 系数矩阵
      Eigen::Matrix<float, 5, 1> matB0;  // 常数项
      Eigen::Vector3f matX0;             // 解向量
      matA0.setZero();
      matB0.fill(-1);  // 平面方程：ax + by + cz + d = 0，这里d=1
      matX0.setZero();

      if (k_indices.size() < 5)
        continue;

      // 构建线性方程组：5个点拟合平面
      for (int j = 0; j < 5; j++)
      {
        matA0(j, 0) = feature_map_ds_.surface_features->points[k_indices[j]].x;
        matA0(j, 1) = feature_map_ds_.surface_features->points[k_indices[j]].y;
        matA0(j, 2) = feature_map_ds_.surface_features->points[k_indices[j]].z;
      }

      // 求解平面方程 ax + by + cz = -1
      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      /// 提取平面参数
      float pa = matX0(0, 0);  // 平面法向量x分量
      float pb = matX0(1, 0);  // 平面法向量y分量
      float pc = matX0(2, 0);  // 平面法向量z分量
      float pd = 1;            // 平面常数项

      /// 归一化平面方程
      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps; pb /= ps; pc /= ps; pd /= ps;

      bool planeValid = true;
      /// 验证平面拟合质量：检查所有点到平面的距离
      for (int j = 0; j < 5; j++)
      {
        if (fabs(pa * feature_map_ds_.surface_features->points[k_indices[j]].x +
                pb * feature_map_ds_.surface_features->points[k_indices[j]].y +
                pc * feature_map_ds_.surface_features->points[k_indices[j]].z +
                pd) > 0.2)  // 距离阈值0.2米
        {
          planeValid = false;
          break;
        }
      }

      if (planeValid)
      {
        // 计算当前点到拟合平面的距离
        float pd2 = pa * point_inM.x + pb * point_inM.y + pc * point_inM.z + pd;

        // 计算权重：距离越近权重越大，同时考虑点的距离原点的距离
        float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(point_inM.x * point_inM.x + point_inM.y * point_inM.y + point_inM.z * point_inM.z));

        // 如果权重过小，跳过
        if (s <= 0.1)
        {
          std::cout << YELLOW << "bad surface association\n" << RESET;
          continue;
        }

        // 构造点对应关系
        PointCorrespondence point_cor;
        point_cor.geo_type = Plane;  // 几何类型：平面
        point_cor.t_point = lf_cur.surface_features->points[i].timestamp;  // 点的时间戳
        point_cor.t_map = feature_map_ds_.timestamp;                       // 地图时间戳
        point_cor.point = Eigen::Vector3d(lf_cur.surface_features->points[i].x,
                                          lf_cur.surface_features->points[i].y,
                                          lf_cur.surface_features->points[i].z);  // 当前点坐标
        point_cor.geo_plane = Eigen::Vector4d(pa, pb, pc, pd);  // 平面方程参数
        point_cor.scale = s;  // 匹配权重
        point_correspondence_.push_back(point_cor);

        // 为可视化添加对应的地图点
        double color = point_cor.point[0] + point_cor.point[1] + point_cor.point[2];
        for (int i = 0; i < 5; ++i)
        {
          VPoint vp;
          vp.x = matA0(i, 0);
          vp.y = matA0(i, 1);
          vp.z = matA0(i, 2);
          vp.intensity = color;
          map_corrs_viewer.push_back(vp);
        }
      }
    }

    int surf_feature_num = point_correspondence_.size() - corner_feature_num;  // 计算表面特征匹配数量
    // LOG(INFO) << "point correspondence: " << point_correspondence_.size()
    //           << "; corner/surfel " << corner_feature_num << "/"
    //           << surf_feature_num;
    return true;
  }

  // TODO 
  void LidarHandler::DownSampleCorrespondence()
  {
    int step = cor_downsample_;
    // if (point_correspondence_.size() < 1200)
    // step = 1;

    size_t reserve_size = point_correspondence_.size() / step + 1;
    point_correspondence_ds_.clear();
    point_correspondence_ds_.reserve(reserve_size);
    for (size_t i = 0; i < point_correspondence_.size(); i += step)
    {
      point_correspondence_ds_.push_back(point_correspondence_.at(i));
    }
  }

  void LidarHandler::FindNearbyKeyFrames(const int key, const int search_num,
                                          PosCloud::Ptr &cloud_in_G) const
  {
    cloud_in_G->clear();

    int keyscan_size = cloud_key_pos_xy_->size();
    for (int i = -search_num; i <= search_num; i++)
    {
      int key_index = key + i;
      if (key_index < 0 || key_index >= keyscan_size)
        continue;

      int64_t kf_time = cloud_key_pos_xy_->points.at(key_index).timestamp;
      if (local_feature_container_all_ds_.find(kf_time) ==
          local_feature_container_all_ds_.end())
        continue;

      const auto &local_lf = local_feature_container_all_ds_.at(kf_time);

      PosCloud local_cloud, global_cloud;
      if (!local_lf.full_cloud->empty())
      {
        local_cloud += *local_lf.full_cloud;
      }
      else
      {
        local_cloud += *local_lf.surface_features;
      }

      Eigen::Matrix4d T_cur_to_G = trajectory_->GetLidarPoseNURBS(kf_time).matrix();
      pcl::transformPointCloud(local_cloud, global_cloud, T_cur_to_G);
      *cloud_in_G += global_cloud;
    }
  }

  void LidarHandler::UpdateCloudKeyPos(
      const std::pair<int, int> &cur_wrt_history)
  {
    // update key_pose
    int64_t t_cur = cloud_key_pos_xy_->points[cur_wrt_history.first].timestamp;
    int64_t t_history =
        cloud_key_pos_xy_->points[cur_wrt_history.second].timestamp;

    for (size_t i = 0; i < cloud_key_pos_xy_->size(); i++)
    {
      int64_t kf_time = cloud_key_pos_xy_->points[i].timestamp;
      if (kf_time < t_history || kf_time > t_cur)
      {
        continue;
      }

      Eigen::Vector3d p = trajectory_->GetLidarPoseNURBS(kf_time).translation();
      cloud_key_pos_->points[i].x = p(0);
      cloud_key_pos_->points[i].y = p(1);
      cloud_key_pos_->points[i].z = p(2);
      cloud_key_pos_xy_->points[i].x = p(0);
      cloud_key_pos_xy_->points[i].y = p(1);
    }

    // 
    key_frame_updated_ = true;
  }

} // namespace cocolic
