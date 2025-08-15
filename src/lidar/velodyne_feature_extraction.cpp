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

#include "velodyne_feature_extraction.h"
#include <cocolic/feature_cloud.h>
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg

using namespace std;

namespace cocolic
{

  VelodyneFeatureExtraction::VelodyneFeatureExtraction(const YAML::Node &node)
      : fea_param_(LiDARFeatureParam(node["VLP16"]))
  {
    n_scan = node["VLP16"]["N_SCAN"].as<int>();
    horizon_scan = node["VLP16"]["Horizon_SCAN"].as<int>();

    pub_corner_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("lidar_feature/corner_cloud", 10);
    pub_surface_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("lidar_feature/surface_cloud", 10);
    pub_full_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("lidar_feature/full_cloud", 10);

    pub_feature_cloud =
        nh.advertise<cocolic::feature_cloud>("lidar_feature/feature_cloud", 10);

    AllocateMemory();
    ResetParameters();
  }

  void VelodyneFeatureExtraction::AllocateMemory()
  {
    p_full_cloud.reset(new RTPointCloud());
    p_full_cloud->resize(n_scan * horizon_scan);

    range_mat = cv::Mat(n_scan, horizon_scan, CV_32F, cv::Scalar::all(FLT_MAX));

    p_extracted_cloud.reset(new RTPointCloud());

    p_corner_cloud.reset(new RTPointCloud());
    p_surface_cloud.reset(new RTPointCloud());

    point_range_list.assign(n_scan * horizon_scan, 0);
    point_column_id.assign(n_scan * horizon_scan, 0);
    start_ring_index.assign(n_scan, 0);
    end_ring_index.assign(n_scan, 0);

    cloud_smoothness.resize(n_scan * horizon_scan);

    cloud_curvature = new float[n_scan * horizon_scan];
    cloud_neighbor_picked = new int[n_scan * horizon_scan];
    cloud_label = new int[n_scan * horizon_scan];

    down_size_filter.SetResolution(fea_param_.odometry_surface_leaf_size);
  }

  void VelodyneFeatureExtraction::ResetParameters()
  {
    p_full_cloud->clear();
    p_full_cloud->resize(n_scan * horizon_scan);
    p_extracted_cloud->clear();
    range_mat = cv::Mat(n_scan, horizon_scan, CV_32F, cv::Scalar::all(FLT_MAX));
  }

  void VelodyneFeatureExtraction::LidarHandler(
      const sensor_msgs::PointCloud2::ConstPtr &lidar_msg)
  {
    RTPointCloud::Ptr cur_cloud(new RTPointCloud());
    bool check_field_passed = ParsePointCloud(lidar_msg, cur_cloud);

    if (!check_field_passed)
      return;

    LidarHandler(cur_cloud);
  }

  /**
   * [功能描述]：机械雷达点云特征提取的主处理函数，将原始点云转换为距离图像并提取角点和表面特征
   * 实现完整的点云预处理、特征分类和提取流程
   * @param raw_cloud：输入的原始RTPointCloud格式点云指针，包含XYZ坐标、强度、环号和时间信息
   */
  void VelodyneFeatureExtraction::LidarHandler(
      const RTPointCloud::Ptr raw_cloud)
  {
    // 重置角点特征点云容器，清除上一帧的角点数据
    p_corner_cloud.reset(new RTPointCloud());
    // 重置表面特征点云容器，清除上一帧的表面特征数据  
    p_surface_cloud.reset(new RTPointCloud());

    // 根据点云组织形式选择不同的距离图像转换方法
    if (raw_cloud->isOrganized())
      // 处理有序点云：直接利用点云的行列结构信息转换为距离图像
      // 有序点云保持了雷达扫描的几何结构，可以高效地转换为距离图像
      OrganizedCloudToRangeImage(raw_cloud, range_mat, p_full_cloud);
    else
      // 处理无序点云：根据点的环号(ring)和方位角重新组织为距离图像
      // 无序点云需要重新计算每个点在距离图像中的位置
      RTCloudToRangeImage(raw_cloud, range_mat, p_full_cloud);

    // 注释说明：此时距离矩阵[range_mat]和完整点云[p_full_cloud]已准备就绪
    
    // 第一步：点云提取和有效性检查
    // 从距离图像中提取有效点云，过滤掉无效点、地面点等
    CloudExtraction();
    
    // 第二步：计算点云平滑度
    // 为每个点计算曲率值，作为特征分类的重要依据
    CaculateSmoothness();
    
    // 第三步：标记遮挡点
    // 识别和标记由于物体遮挡造成的不连续边界点，避免错误特征提取
    MarkOccludedPoints();
    
    // 第四步：特征提取
    // 根据曲率值将点分类为角点特征（高曲率）和表面特征（低曲率）
    ExtractFeatures();
    
    // 第五步：重置参数
    // 清理临时变量和缓存，为处理下一帧点云做准备
    ResetParameters();

    // 发布处理后的点云数据到"map"坐标系，用于可视化和调试
    PublishCloud("map");
  }

  bool VelodyneFeatureExtraction::CheckMsgFields(
      const sensor_msgs::PointCloud2 &cloud_msg, std::string fields_name)
  {
    bool flag = false;
    for (size_t i = 0; i < cloud_msg.fields.size(); ++i)
    {
      if (cloud_msg.fields[i].name == fields_name)
      {
        flag = true;
        break;
      }
    }

    // if (!flag) {
    //   LOG(WARNING) << "PointCloud2 channel [" << fields_name
    //                << "] not available, please configure your point cloud
    //                data!";
    //   // ros::shutdown();
    // }
    return flag;
  }

  /**
   * [功能描述]：解析不同类型机械雷达的PointCloud2消息，自动识别消息格式并转换为统一的RTPointCloud格式
   * 支持Velodyne、Ouster、Hesai等品牌雷达的数据格式自动适配
   * @param lidar_msg：输入的雷达PointCloud2消息常量智能指针
   * @param out_cloud：输出的RTPointCloud格式点云指针，统一的内部点云表示
   * @return 布尔值，表示消息解析是否成功（字段检查是否通过）
   */
  bool VelodyneFeatureExtraction::ParsePointCloud(
      const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud) const
  {
    // 静态变量：用于记录首次消息字段检查的结果，避免重复检查提高效率
    static bool has_checked = false;          // 是否已经进行过字段检查
    static bool check_field_passed = false;  // 字段检查是否通过标志
    static bool has_t_field = false;         // 是否有"t"字段（Ouster雷达使用）
    static bool has_time_field = false;      // 是否有"time"字段（Velodyne雷达使用）
    static bool has_timestamp_field = false; // 是否有"timestamp"字段（Hesai雷达使用）

    /// 首次消息字段检查：识别雷达类型和时间戳字段格式
    if (!has_checked)
    {
      // 设置检查完成标志，确保只检查一次
      has_checked = true;
      // 检查是否存在"ring"字段，所有支持的雷达都应该有此字段
      bool has_ring_field = CheckMsgFields(*lidar_msg, "ring");
      // 检查Velodyne雷达的时间字段："time"（float类型，秒为单位）
      // 适用于LVI-SAM、LIO-SAM等数据集
      has_time_field = CheckMsgFields(*lidar_msg, "time");
      // 检查Ouster雷达的时间字段："t"（uint32_t类型，纳秒为单位）
      // 适用于VIRAL等数据集
      has_t_field = CheckMsgFields(*lidar_msg, "t");
      // 检查Hesai雷达的时间字段："timestamp"（float类型，秒为单位）
      // 适用于Hesai PandarQT等雷达
      has_timestamp_field = CheckMsgFields(*lidar_msg, "timestamp");

      // 综合字段检查结果：必须有ring字段，且至少有一种时间字段
      check_field_passed = has_ring_field && (has_time_field || has_t_field || has_timestamp_field);

      // 调试信息（已注释）：用于提示缺失的字段信息
      // if (!has_ring_field)
      //   LOG(WARNING) << "[ParsePointCloud] input cloud NOT has [ring] field";
      
      // if (!has_time_field && !has_t_field && !has_timestamp_field)
      //   LOG(WARNING)
      //       << "[ParsePointCloud] input cloud NOT has [time] or [t] or [timestamp] field";
    }

    /// 根据检查结果进行点云格式转换
    if (check_field_passed)
    {
      // 处理Velodyne雷达格式：使用"time"字段（float类型秒时间戳）
      if (has_time_field)
      {
        // 创建Velodyne临时点云容器（RTPointTmp格式）
        RTPointCloudTmp::Ptr tmp_out_cloud(new RTPointCloudTmp);
        // 将ROS消息转换为PCL点云格式
        pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

        // 将临时点云转换为统一的RTPointCloud格式（时间字段从float转为double）
        RTPointCloudTmp2RTPointCloud(tmp_out_cloud, out_cloud);
      }
      // 处理Ouster雷达格式：使用"t"字段（uint32_t类型纳秒时间戳）
      else if (has_t_field)
      {
        // 创建Ouster临时点云容器（OusterPointTmp格式）
        OusterPointCloudTmp::Ptr tmp_out_cloud(new OusterPointCloudTmp());
        // 将ROS消息转换为PCL点云格式
        pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

        // 将Ouster点云转换为统一的RTPointCloud格式（时间单位从纳秒转为秒）
        OusterPointCloudTmp2RTPointCloud(tmp_out_cloud, out_cloud);
      }
      // 处理Hesai雷达格式：使用"timestamp"字段（double类型秒时间戳）
      else if (has_timestamp_field)
      {
        // 创建Hesai临时点云容器（RTPointTmpHesai格式）
        RTPointCloudTmpHesai::Ptr tmp_out_cloud(new RTPointCloudTmpHesai);
        // 将ROS消息转换为PCL点云格式
        pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

        // 将Hesai点云转换为统一的RTPointCloud格式
        RTPointCloudTmp2RTPointCloudHesai(tmp_out_cloud, out_cloud);
      }
    }
    // 返回字段检查结果，表示消息解析是否成功
    return check_field_passed;
  }

  bool VelodyneFeatureExtraction::ParsePointCloudNoFeature(
      const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud)
  {
    RTPointCloudTmp::Ptr tmp_out_cloud(new RTPointCloudTmp);
    pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

    int plsize = tmp_out_cloud->points.size();

    p_corner_cloud.reset(new RTPointCloud());
    p_surface_cloud.reset(new RTPointCloud());
    p_full_cloud.reset(new RTPointCloud());

    // p_corner_cloud->reserve(plsize);
    // p_surface_cloud->reserve(plsize);
    // p_full_cloud->resize(plsize);

    std::cout << "[plsize] " << plsize << std::endl;

    RTPointCloudTmp2RTPointCloud(tmp_out_cloud, out_cloud);

    for (int i = 0; i < plsize; i++)
    {
      const auto &pt = tmp_out_cloud->points[i];
      RTPoint added_pt;
      added_pt.x = pt.x;
      added_pt.y = pt.y;
      added_pt.z = pt.z;
      added_pt.intensity = pt.intensity;
      added_pt.ring = pt.ring;
      added_pt.time = int64_t(pt.time * 1e9);

      if (i % 4 == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (2.0 * 2.0))
        {
            // p_surface_cloud->points.push_back(added_pt);
            p_surface_cloud->push_back(added_pt);
        }
      }
    }

    /// 
    p_full_cloud->push_back((*p_surface_cloud)[0]);
    p_corner_cloud->push_back((*p_surface_cloud)[0]);

    return true;
  }

  void VelodyneFeatureExtraction::OrganizedCloudToRangeImage(
      const RTPointCloud::Ptr cur_cloud, cv::Mat &dist_image,
      RTPointCloud::Ptr &corresponding_cloud) const
  {
    for (size_t column_id = 0; column_id < cur_cloud->width; ++column_id)
    {
      for (size_t row_id = 0; row_id < cur_cloud->height; ++row_id)
      {
        const RTPoint &p = cur_cloud->at(column_id, row_id);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;

        if (row_id < 0 || row_id >= size_t(n_scan))
          continue;
        if (column_id < 0 || column_id >= size_t(horizon_scan))
          continue;

        float range = pcl::PointNorm<RTPoint>(p);
        if (range < fea_param_.min_distance || range > fea_param_.max_distance)
          continue;
        if (dist_image.at<float>(row_id, column_id) != FLT_MAX)
          continue;
        dist_image.at<float>(row_id, column_id) = range;

        /// 
        int index = column_id + row_id * horizon_scan;
        corresponding_cloud->points[index] = p;
      }
    }
  }

  void VelodyneFeatureExtraction::RTCloudToRangeImage(
      const RTPointCloud::Ptr cur_cloud, cv::Mat &dist_image,
      RTPointCloud::Ptr &corresponding_cloud) const
  {
    static float angle_resolution = 360.0 / float(horizon_scan);
    static float rad2deg = 180.0 / M_PI;

    for (const RTPoint &p : cur_cloud->points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      int row_id = p.ring;
      if (row_id < 0 || row_id >= n_scan)
        continue;
      float horizon_angle = atan2(p.x, p.y) * rad2deg;

      int column_id =
          -round((horizon_angle - 90.0) / angle_resolution) + horizon_scan / 2;
      if (column_id >= horizon_scan)
        column_id -= horizon_scan;
      if (column_id < 0 || column_id >= horizon_scan)
        continue;

      float range = pcl::PointNorm<RTPoint>(p);
      if (range < fea_param_.min_distance || range > fea_param_.max_distance)
        continue;

      if (dist_image.at<float>(row_id, column_id) != FLT_MAX)
        continue;
      dist_image.at<float>(row_id, column_id) = range;

      int index = column_id + row_id * horizon_scan;
      corresponding_cloud->points[index] = p;
    }
  }

  void VelodyneFeatureExtraction::CloudExtraction()
  {
    int point_index = 0;
    for (int i = 0; i < n_scan; i++)
    {
      start_ring_index[i] = point_index - 1 + 5;
      for (int j = 0; j < horizon_scan; j++)
      {
        if (range_mat.at<float>(i, j) != FLT_MAX)
        {
          // mark the points' column index for marking occlusion later
          point_column_id[point_index] = j;
          // save range info
          point_range_list[point_index] = range_mat.at<float>(i, j);
          // save extracted cloud
          p_extracted_cloud->push_back(
              p_full_cloud->points[j + i * horizon_scan]);
          // size of extracted cloud
          point_index++;
        }
      }
      end_ring_index[i] = point_index - 1 - 5;
    }
  }

  void VelodyneFeatureExtraction::CaculateSmoothness()
  {
    for (size_t i = 5; i < p_extracted_cloud->points.size() - 5; i++)
    {
      float diff_range = point_range_list[i - 5] + point_range_list[i - 4] +
                         point_range_list[i - 3] + point_range_list[i - 2] +
                         point_range_list[i - 1] - point_range_list[i] * 10 +
                         point_range_list[i + 1] + point_range_list[i + 2] +
                         point_range_list[i + 3] + point_range_list[i + 4] +
                         point_range_list[i + 5];
      cloud_curvature[i] = diff_range * diff_range;
      cloud_neighbor_picked[i] = 0;
      cloud_label[i] = 0;
      cloud_smoothness[i].value = cloud_curvature[i];
      cloud_smoothness[i].ind = i;
    }
  }

  void VelodyneFeatureExtraction::MarkOccludedPoints()
  {
    for (size_t i = 5; i < p_extracted_cloud->points.size() - 6; i++)
    {
      float depth1 = point_range_list[i];
      float depth2 = point_range_list[i + 1];
      int column_diff =
          std::abs(int(point_column_id[i + 1] - point_column_id[i]));

      if (column_diff < 10)
      {
        if (depth1 - depth2 > 0.3)
        {
          cloud_neighbor_picked[i - 5] = 1;
          cloud_neighbor_picked[i - 4] = 1;
          cloud_neighbor_picked[i - 3] = 1;
          cloud_neighbor_picked[i - 2] = 1;
          cloud_neighbor_picked[i - 1] = 1;
          cloud_neighbor_picked[i] = 1;
        }
        else if (depth2 - depth1 > 0.3)
        {
          cloud_neighbor_picked[i + 1] = 1;
          cloud_neighbor_picked[i + 2] = 1;
          cloud_neighbor_picked[i + 3] = 1;
          cloud_neighbor_picked[i + 4] = 1;
          cloud_neighbor_picked[i + 5] = 1;
          cloud_neighbor_picked[i + 6] = 1;
        }
      }

      float diff1 =
          std::abs(float(point_range_list[i - 1] - point_range_list[i]));
      float diff2 =
          std::abs(float(point_range_list[i + 1] - point_range_list[i]));

      if (diff1 > 0.02 * point_range_list[i] &&
          diff2 > 0.02 * point_range_list[i])
        cloud_neighbor_picked[i] = 1;
    }
  }

  void VelodyneFeatureExtraction::ExtractFeatures()
  {
    RTPointCloud::Ptr surface_cloud_scan(new RTPointCloud());
    RTPointCloud::Ptr surface_cloud_scan_downsample(new RTPointCloud());

    for (int i = 0; i < n_scan; i++)
    {
      surface_cloud_scan->clear();

      /// 
      for (int j = 0; j < 6; j++)
      {
        int sp = (start_ring_index[i] * (6 - j) + end_ring_index[i] * j) / 6;
        int ep =
            (start_ring_index[i] * (5 - j) + end_ring_index[i] * (j + 1)) / 6 - 1;
        if (sp >= ep)
          continue;
        std::sort(cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep,
                  by_value());

        /// 
        int largest_picked_num = 0;
        for (int k = ep; k >= sp; k--)
        {
          int index = cloud_smoothness[k].ind;
          if (cloud_neighbor_picked[index] == 0 &&
              cloud_curvature[index] > fea_param_.edge_threshold)
          {
            largest_picked_num++;
            if (largest_picked_num <= 20)
            {
              cloud_label[index] = 1;
              p_corner_cloud->push_back(p_extracted_cloud->points[index]);
            }
            else
            {
              break;
            }

            cloud_neighbor_picked[index] = 1;
            for (int l = 1; l <= 5; l++)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l - 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l + 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
          }
        }

        /// 
        for (int k = sp; k <= ep; k++)
        {
          int index = cloud_smoothness[k].ind;
          if (cloud_neighbor_picked[index] == 0 &&
              cloud_curvature[index] < fea_param_.surf_threshold)
          {
            cloud_label[index] = -1;
            cloud_neighbor_picked[index] = 1;

            for (int l = 1; l <= 5; l++)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l - 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l + 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
          }
        }
        for (int k = sp; k <= ep; k++)
        {
          if (cloud_label[k] <= 0)
          {
            surface_cloud_scan->push_back(p_extracted_cloud->points[k]);
          }
        }
      }

      surface_cloud_scan_downsample->clear();
      down_size_filter.SetInputCloud(surface_cloud_scan);
      down_size_filter.Filter(surface_cloud_scan_downsample);
      *p_surface_cloud += *surface_cloud_scan_downsample;
    }
  }

  void VelodyneFeatureExtraction::PublishCloud(std::string frame_id)
  {
    bool pub_fea = (pub_full_cloud.getNumSubscribers() != 0);

    cocolic::feature_cloud feature_msg;
    if (pub_fea || pub_corner_cloud.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 corner_msg;
      pcl::toROSMsg(*p_corner_cloud, corner_msg);
      corner_msg.header.stamp = ros::Time::now();
      corner_msg.header.frame_id = frame_id;

      pub_corner_cloud.publish(corner_msg);
      feature_msg.corner_cloud = corner_msg;
    }
    if (pub_fea || pub_surface_cloud.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 surface_msg;
      pcl::toROSMsg(*p_surface_cloud, surface_msg);
      surface_msg.header.stamp = ros::Time::now();
      surface_msg.header.frame_id = frame_id;

      pub_surface_cloud.publish(surface_msg);
      feature_msg.surface_cloud = surface_msg;
    }

    if (pub_fea || pub_full_cloud.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 full_msg;
      pcl::toROSMsg(*p_full_cloud, full_msg);
      full_msg.header.stamp = ros::Time::now();
      full_msg.header.frame_id = frame_id;

      pub_full_cloud.publish(full_msg);
      feature_msg.full_cloud = full_msg;
    }

    if (pub_fea)
    {
      feature_msg.header.stamp = ros::Time::now();
      feature_msg.header.frame_id = frame_id;

      pub_feature_cloud.publish(feature_msg);
    }

    //  rosbag::Bag bagWrite;
    //  bagWrite.open("/home/ha/rosbag/liso-bag/simu_bag/sim_feature.bag",
    //  rosbag::bagmode::Append); bagWrite.write("/feature_cloud",
    //  feature_msg.header.stamp, feature_msg); bagWrite.close();
  }

} // namespace cocolic
