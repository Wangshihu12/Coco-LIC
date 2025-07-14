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

#include <eigen_conversions/eigen_msg.h>
#include <odom/odometry_manager.h>
#include <numeric>

#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>

std::fstream rgb_file;
std::fstream img_file;

namespace cocolic
{

  // OdometryManager 构造函数：初始化激光雷达-惯性-相机里程计系统
  OdometryManager::OdometryManager(const YAML::Node &node, ros::NodeHandle &nh)
  : odometry_mode_(LIO), is_initialized_(false)  // 默认里程计模式为LIO，初始化状态为false
  {
  // 获取配置文件路径
  std::string config_path;
  nh.param<std::string>("project_path", config_path, "");
  config_path += "/config";

  // 加载激光雷达配置文件
  std::string lidar_yaml = node["lidar_yaml"].as<std::string>();
  YAML::Node lidar_node = YAML::LoadFile(config_path + lidar_yaml);

  // 加载IMU配置文件
  std::string imu_yaml = node["imu_yaml"].as<std::string>();
  YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);

  // 加载相机配置文件
  std::string cam_yaml = config_path + node["camera_yaml"].as<std::string>();
  YAML::Node cam_node = YAML::LoadFile(cam_yaml);

  // 设置里程计模式并输出提示信息
  odometry_mode_ = OdometryMode(node["odometry_mode"].as<int>());
  std::cout << "\n🥥 Odometry Mode: ";
  if (odometry_mode_ == LICO)
  {
  std::cout << "LiDAR-Inertial-Camera Odometry 🥥" << std::endl;  // 激光雷达-惯性-相机里程计
  }
  else if (odometry_mode_ == LIO)
  {
  std::cout << "LiDAR-Inertial Odometry 🥥" << std::endl;  // 激光雷达-惯性里程计
  }

  // 外参设置：传感器到IMU的变换矩阵
  ExtrinsicParam EP_LtoI, EP_CtoI, EP_ItoI, EP_MtoI;
  EP_LtoI.Init(lidar_node["lidar0"]["Extrinsics"]);  // 激光雷达到IMU的外参
  if (odometry_mode_ == LICO)  // 只有在LICO模式下才初始化相机外参
  EP_CtoI.Init(cam_node["CameraExtrinsics"]);  // 相机到IMU的外参
  if (node["IMUExtrinsics"])  // 如果存在IMU外参配置
  EP_ItoI.Init(imu_node["IMUExtrinsics"]);  // IMU到IMU的外参（多IMU情况）
  EP_MtoI.Init(imu_node["MarkerExtrinsics"]);  // 标记点到IMU的外参

  // 初始化轨迹对象，使用B样条进行轨迹参数化
  trajectory_ = std::make_shared<Trajectory>(-1, 0);
  trajectory_->SetSensorExtrinsics(SensorType::LiDARSensor, EP_LtoI);  // 设置激光雷达外参
  trajectory_->SetSensorExtrinsics(SensorType::CameraSensor, EP_CtoI);  // 设置相机外参
  trajectory_->SetSensorExtrinsics(SensorType::IMUSensor, EP_ItoI);  // 设置IMU外参
  trajectory_->SetSensorExtrinsics(SensorType::Marker, EP_MtoI);  // 设置标记点外参

  // 非均匀B样条相关参数设置
  t_add_ = node["t_add"].as<double>();  // 时间增量
  t_add_ns_ = t_add_ * S_TO_NS;  // 转换为纳秒
  non_uniform_ = node["non_uniform"].as<bool>();  // 是否使用非均匀B样条
  distance0_ = node["distance0"].as<double>();  // 初始距离参数

  // 激光雷达处理器初始化
  lidar_iter_ = node["lidar_iter"].as<int>();  // 激光雷达迭代次数
  use_lidar_scale_ = node["use_lidar_scale"].as<bool>();  // 是否使用激光雷达尺度
  lidar_handler_ = std::make_shared<LidarHandler>(lidar_node, trajectory_);  // 创建激光雷达处理器
  std::cout << "\n🍺 The number of multiple LiDARs is " << lidar_node["num_lidars"].as<int>() << "." << std::endl;

  // IMU初始化器
  imu_initializer_ = std::make_shared<IMUInitializer>(imu_node);  // 创建IMU初始化器
  gravity_norm_ = imu_initializer_->GetGravity().norm();  // 获取重力向量的模长

  // 相机处理器初始化
  camera_handler_ = std::make_shared<R3LIVE>(cam_node, EP_CtoI);  // 创建相机处理器
  t_begin_add_cam_ = node["t_begin_add_cam"].as<double>() * S_TO_NS;  // 开始添加相机数据的时间
  v_points_.clear();  // 清空3D点容器
  px_obss_.clear();  // 清空像素观测容器

  // 提取相机内参
  double fx = cam_node["cam_fx"].as<double>();  // 焦距x
  double fy = cam_node["cam_fy"].as<double>();  // 焦距y
  double cx = cam_node["cam_cx"].as<double>();  // 主点x
  double cy = cam_node["cam_cy"].as<double>();  // 主点y
  // 构建相机内参矩阵
  K_ << fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0;

  // 轨迹管理器初始化：使用B样条参数化轨迹
  trajectory_manager_ = std::make_shared<TrajectoryManager>(node, config_path, trajectory_);
  trajectory_manager_->use_lidar_scale = use_lidar_scale_;  // 设置是否使用激光雷达尺度
  trajectory_manager_->SetIntrinsic(K_);  // 设置相机内参

  // 设置B样条控制点分割参数
  int division_coarse = node["division_coarse"].as<int>();  // 粗分割数
  cp_add_num_coarse_ = division_coarse;  // 保存粗分割控制点数量
  trajectory_manager_->SetDivisionParam(division_coarse, -1);  // 设置分割参数

  // 初始化里程计可视化器
  odom_viewer_.SetPublisher(nh);

  // 消息管理器初始化：负责加载rosbag数据
  msg_manager_ = std::make_shared<MsgManager>(node, config_path, nh);

  // 设置调试和可视化参数
  bool verbose;
  nh.param<double>("pasue_time", pasue_time_, -1);  // 暂停时间
  nh.param<bool>("verbose", verbose, false);  // 是否输出详细信息
  trajectory_manager_->verbose = verbose;  // 设置轨迹管理器的详细输出模式

  // 评估相关参数
  is_evo_viral_ = node["is_evo_viral"].as<bool>();  // 是否使用viral数据集评估
  CreateCacheFolder(config_path, msg_manager_->bag_path_);  // 创建缓存文件夹

  // 高斯-LIC相关参数
  if_3dgs_ = node["if_3dgs"].as<bool>();  // 是否启用3D高斯溅射
  lidar_skip_ = node["lidar_skip"].as<int>();  // 激光雷达点跳跃间隔

  // 设置输出精度
  std::cout << std::fixed << std::setprecision(4);
  // LOG(INFO) << std::fixed << std::setprecision(4);
  }

  bool OdometryManager::CreateCacheFolder(const std::string &config_path,
                                          const std::string &bag_path)
  {
    boost::filesystem::path path_cfg(config_path);
    boost::filesystem::path path_bag(bag_path);
    if (path_bag.extension() != ".bag")
    {
      return false;
    }
    std::string bag_name_ = path_bag.stem().string();

    std::string cache_path_parent_ = path_cfg.parent_path().string();
    cache_path_ = cache_path_parent_ + "/data/" + bag_name_;
    // boost::filesystem::create_directory(cache_path_);
    return true;
  }

  // 运行rosbag数据处理的主循环函数
  void OdometryManager::RunBag()
  {
    while (ros::ok())  // ROS系统运行时持续处理
    {
      /// [1] 处理新到达的数据帧：激光雷达、IMU或相机数据
      msg_manager_->SpinBagOnce();  // 从rosbag中读取一帧数据
      if (!msg_manager_->has_valid_msg_)  // 如果没有有效数据
      {
        break;  // 退出循环，数据处理完毕
      }

      /// [2] 静态初始化，系统启动时不要移动！
      if (!is_initialized_)  // 如果系统尚未初始化
      {
        // 将所有IMU数据送入初始化器
        while (!msg_manager_->imu_buf_.empty())
        {
          imu_initializer_->FeedIMUData(msg_manager_->imu_buf_.front());  // 喂入IMU数据
          msg_manager_->imu_buf_.pop_front();  // 移除已处理的数据
        }

        // 尝试进行静态初始化
        if (imu_initializer_->StaticInitialIMUState())  // 如果静态初始化成功
        {
          SetInitialState();  // 设置初始状态
          std::cout << "\n🍺 Static initialization succeeds.\n";  // 输出成功信息
          std::cout << "\n🍺 Trajectory start time: " << trajectory_->GetDataStartTime() << " ns.\n";
        }
        else
        {
          continue;  // 初始化失败，继续等待更多数据
        }
      }

      /// [3] 为最新时间间隔delta_t准备数据
      static bool is_two_seg_prepared = false;  // 标记是否已准备好两段数据
      static int seg_msg_cnt = 0;  // 已准备的段数计数器
      if (!is_two_seg_prepared)  // 如果两段数据尚未准备好
      {
        if (PrepareTwoSegMsgs(seg_msg_cnt))  // 准备interval0和interval1
        {
          seg_msg_cnt++;  // 成功准备一段，计数器增加
        }
        if (seg_msg_cnt == 2)  // 如果interval0和interval1都准备好了
        {
          is_two_seg_prepared = true;  // 标记两段数据已准备完毕
          UpdateTwoSeg();  // 更新两段轨迹
          trajectory_->InitBlendMat();  // 初始化混合矩阵，由B样条的节点计算得出
        }
        else
        {
          continue;  // 两段数据尚未完全准备好，继续下一循环
        }
      }

      /// [4] 更新最新时间间隔delta_t内的轨迹段
      if (PrepareMsgs())  // 准备当前处理的消息
      {
        // 根据IMU数据决定时间间隔delta_t内的控制点布局
        UpdateOneSeg();  // 更新一个轨迹段
        
        // 计算控制点偏移量
        int offset = cp_add_num_cur + cp_add_num_next + cp_add_num_next_next;
        
        // 为当前段添加混合矩阵
        for (int i = 0; i < cp_add_num_cur; i++)
        {
          trajectory_->AddBlendMat(offset - i);  // 混合矩阵由B样条的节点计算得出
        }
        
        trajectory_manager_->SetDivision(cp_add_num_cur);  // 设置分割参数
        
        // 设置轨迹起始索引，2作为边界或容差
        trajectory_->startIdx = trajectory_->knts.size() - 1 - offset - 2;
        if (trajectory_->startIdx < 0)
        {
          trajectory_->startIdx = 0;  // 确保索引不为负
        }

        // 融合激光雷达-IMU-相机数据来更新轨迹
        SolveLICO();

        // 深拷贝：更新消息缓存的滑动窗口
        msg_manager_->cur_msgs = NextMsgs();  // 重置当前消息
        msg_manager_->cur_msgs = msg_manager_->next_msgs;  // 当前=下一个
        msg_manager_->cur_msgs.image = msg_manager_->next_msgs.image.clone();  // 深拷贝图像
        
        msg_manager_->next_msgs = NextMsgs();  // 重置下一个消息
        msg_manager_->next_msgs = msg_manager_->next_next_msgs;  // 下一个=下下个
        msg_manager_->next_msgs.image = msg_manager_->next_next_msgs.image.clone();  // 深拷贝图像
        
        msg_manager_->next_next_msgs = NextMsgs();  // 重置下下个消息

        // 更新时间戳：滑动窗口向前移动
        traj_max_time_ns_cur = traj_max_time_ns_next;
        traj_max_time_ns_next = traj_max_time_ns_next_next;
        
        // 更新控制点数量：滑动窗口向前移动
        cp_add_num_cur = cp_add_num_next;
        cp_add_num_next = cp_add_num_next_next;

        // 限制图像缓存大小，防止内存溢出
        while (msg_manager_->image_buf_.size() > 10)
        {
          msg_manager_->image_buf_.pop_front();  // 移除最旧的图像
        }
      }
    }
  }

  // 求解LiDAR-Inertial-Camera里程计：多传感器融合优化的主函数
  void OdometryManager::SolveLICO()
  {
    // 记录当前处理的消息信息（用于调试和日志）
    msg_manager_->LogInfo();
    
    // 检查激光雷达数据的有效性
    if (msg_manager_->cur_msgs.lidar_timestamp < 0)
    {
      // 如果激光雷达时间戳无效，无法进行LiDAR-Inertial-Camera优化
      // LOG(INFO) << "CANT SolveLICO!";
    }

    // LIC优化：融合激光雷达、IMU、相机数据进行轨迹优化
    ProcessLICData();

    // 先验信息更新：基于当前的点云配准结果更新轨迹的先验约束
    trajectory_manager_->UpdateLICPrior(
        lidar_handler_->GetPointCorrespondence());  // 传入激光雷达点对应关系

    // 清理旧数据：移除已处理的IMU数据，保持内存使用效率
    auto &msg = msg_manager_->cur_msgs;  // 获取当前消息的引用
    trajectory_manager_->UpdateLiDARAttribute(msg.lidar_timestamp,      // 激光雷达起始时间
                                              msg.lidar_max_timestamp);  // 激光雷达结束时间
  }

  // 处理LiDAR-Inertial-Camera数据：多传感器融合的核心处理函数
  void OdometryManager::ProcessLICData()
  {
    auto &msg = msg_manager_->cur_msgs;  // 获取当前消息（注意：时间戳为-1的虚假点此时仍存在）
    msg.CheckData();  // 检查数据完整性

    // 判断是否需要处理图像：有图像数据且时间戳超过开始添加相机的时间
    bool process_image = msg.if_have_image && msg.image_timestamp > t_begin_add_cam_;
    if (process_image)
    {
      // 输出处理信息：包含激光雷达扫描数和图像时间
      // LOG(INFO) << "Process " << msg.scan_num << " scans in ["
      //           << msg.lidar_timestamp * NS_TO_S << ", " << msg.lidar_max_timestamp * NS_TO_S << "]"
      //           << "; image_time: " << msg.image_timestamp * NS_TO_S;
    }
    else
    {
      // 输出处理信息：仅激光雷达数据
      // LOG(INFO) << "Process " << msg.scan_num << " scans in ["
      //           << msg.lidar_timestamp * NS_TO_S << ", " << msg.lidar_max_timestamp * NS_TO_S << "]";
    }

    /// [1] 转换激光雷达点云格式 -> feature_cur_、feature_cur_ds_
    lidar_handler_->FeatureCloudHandler(msg.lidar_timestamp, msg.lidar_max_timestamp,
                                    msg.lidar_corner_cloud, msg.lidar_surf_cloud, msg.lidar_raw_cloud);  // 移除虚假点

    /// [2] 基于先验信息和IMU数据粗略优化轨迹（作为良好的初值）
    trajectory_manager_->PredictTrajectory(msg.lidar_timestamp, msg.lidar_max_timestamp,
                                          traj_max_time_ns_cur, cp_add_num_cur, non_uniform_);

    /// [3] 更新激光雷达局部地图
    int active_idx = trajectory_->numKnots() - 1 - cp_add_num_cur - 2;  // 计算活跃节点索引
    trajectory_->SetActiveTime(trajectory_->knts[active_idx]);  // 设置活跃时间
    lidar_handler_->UpdateLidarSubMap();  // 更新激光雷达子地图

    /// [4] 更新视觉局部地图（为当前图像帧跟踪地图点）
    // 更新后：m_map_rgb_pts_in_last_frame_pos = m_map_rgb_pts_in_current_frame_pos
    v_points_.clear();  // 清空3D点容器
    px_obss_.clear();   // 清空像素观测容器
    if (process_image)
    {
      // 获取当前图像时刻的相机位姿
      SE3d Twc = trajectory_->GetCameraPoseNURBS(msg.image_timestamp);
      // 更新视觉子地图
      camera_handler_->UpdateVisualSubMap(msg.image, msg.image_timestamp * NS_TO_S, 
                                        Twc.unit_quaternion(), Twc.translation());
      
      // 提取跟踪到的地图点和对应的像素观测
      auto &map_rgb_pts_in_last_frame_pos = camera_handler_->op_track.m_map_rgb_pts_in_last_frame_pos;
      for (auto it = map_rgb_pts_in_last_frame_pos.begin(); it != map_rgb_pts_in_last_frame_pos.end(); it++)
      {
        RGB_pts *rgb_pt = ((RGB_pts *)it->first);  // 获取RGB地图点
        // 添加3D点坐标
        v_points_.push_back(Eigen::Vector3d(rgb_pt->get_pos()(0, 0), rgb_pt->get_pos()(1, 0), rgb_pt->get_pos()(2, 0)));
        // 添加对应的像素观测
        px_obss_.push_back(Eigen::Vector2d(it->second.x, it->second.y));
      }

      // 可视化：发布跟踪图像和视觉子地图（如果有订阅者）
      if (odom_viewer_.pub_track_img_.getNumSubscribers() != 0 || odom_viewer_.pub_sub_visual_map_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();  // 复制当前图像
        VPointCloud visual_sub_map_debug;  // 光流 + RANSAC *2 -> 3D关联（红色）
        visual_sub_map_debug.clear();

        for (auto it = map_rgb_pts_in_last_frame_pos.begin(); it != map_rgb_pts_in_last_frame_pos.end(); it++)
        {
          RGB_pts *rgb_pt = ((RGB_pts *)it->first);
          // 在图像上画绿色圆圈：光流 + RANSAC *2 -> 2D关联（绿色）
          cv::circle(img_debug, it->second, 2, cv::Scalar(0, 255, 0), -1, 8);
          
          // 构建视觉地图点
          VPoint temp_map;
          temp_map.x = rgb_pt->get_pos()(0, 0);
          temp_map.y = rgb_pt->get_pos()(1, 0);
          temp_map.z = rgb_pt->get_pos()(2, 0);
          temp_map.intensity = 0.;
          visual_sub_map_debug.push_back(temp_map);
        }

        // 发布跟踪图像
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img_debug;
        odom_viewer_.PublishTrackImg(out_msg.toImageMsg());
        odom_viewer_.PublishSubVisualMap(visual_sub_map_debug);
      }
    }

    /// [5] 基于先验、激光雷达、IMU、相机数据精细优化轨迹
    for (int iter = 0; iter < lidar_iter_; ++iter)  // 多次迭代优化
    {
      lidar_handler_->GetLoamFeatureAssociation();  // 获取LOAM特征关联

      if (process_image)  // 如果处理图像
      {
        // 使用LiDAR-Inertial-Camera数据更新轨迹
        trajectory_manager_->UpdateTrajectoryWithLIC(
            iter, msg.image_timestamp,
            lidar_handler_->GetPointCorrespondence(), v_points_, px_obss_, 8);
      }
      else  // 仅使用LiDAR-Inertial数据
      {
        // 使用LiDAR-Inertial数据更新轨迹（空的视觉观测）
        trajectory_manager_->UpdateTrajectoryWithLIC(
            iter, msg.image_timestamp,
            lidar_handler_->GetPointCorrespondence(), {}, {}, 8);
        trajectory_manager_->SetProcessCurImg(false);  // 标记不处理当前图像
      }
    }
    PublishCloudAndTrajectory();  // 发布点云和轨迹

    /// [6] 更新视觉全局地图
    PosCloud::Ptr cloud_undistort = PosCloud::Ptr(new PosCloud);  // 去畸变点云
    auto latest_feature_before_active_time = lidar_handler_->GetFeatureCurrent();  // 获取当前特征
    PosCloud::Ptr cloud_distort = latest_feature_before_active_time.surface_features;  // 畸变的表面特征
    if (cloud_distort->size() != 0)
    {
      // 在全局坐标系下去除点云运动畸变
      trajectory_->UndistortScanInG(*cloud_distort, latest_feature_before_active_time.timestamp, *cloud_undistort);
      // 使用去畸变的点云更新视觉全局地图
      camera_handler_->UpdateVisualGlobalMap(cloud_undistort, latest_feature_before_active_time.time_max * NS_TO_S);
    }

    /// [7] 为当前图像帧关联新的地图点
    if (process_image)
    {
      SE3d Twc = trajectory_->GetCameraPoseNURBS(msg.image_timestamp);  // 获取相机位姿
      // 关联新的地图点到当前图像
      camera_handler_->AssociateNewPointsToCurrentImg(Twc.unit_quaternion(), Twc.translation());

      // 可视化：在当前图像中显示去畸变的激光雷达扫描点
      if (odom_viewer_.pub_undistort_scan_in_cur_img_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        {
          for (int i = 0; i < cloud_undistort->points.size(); i++)
          {
            auto pt = cloud_undistort->points[i];
            Eigen::Vector3d pt_e(pt.x, pt.y, pt.z);  // 世界坐标系下的点
            Eigen::Matrix3d Rwc = Twc.unit_quaternion().toRotationMatrix();  // 相机旋转矩阵
            Eigen::Vector3d twc = Twc.translation();  // 相机平移向量
            // 将世界坐标点转换到相机坐标系
            Eigen::Vector3d pt_cam = Rwc.transpose() * pt_e - Rwc.transpose() * twc;
            double X = pt_cam.x(), Y = pt_cam.y(), Z = pt_cam.z();
            // 投影到图像平面
            cv::Point2f pix(K_(0, 0) * X / Z + K_(0, 2), K_(1, 1) * Y / Z + K_(1, 2));
            cv::circle(img_debug, pix, 2, cv::Scalar(0, 0, 255), -1, 8);  // 红色圆圈
          }
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = ros::Time::now();
          out_msg.encoding = sensor_msgs::image_encodings::BGR8;
          out_msg.image = img_debug;
          odom_viewer_.PublishUndistortScanInCurImg(out_msg.toImageMsg());
        }
      }

      // 可视化：显示当前图像中的新旧地图点
      if (odom_viewer_.pub_old_and_new_added_points_in_cur_img_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        auto obss = camera_handler_->op_track.m_map_rgb_pts_in_last_frame_pos;
        for (auto it = obss.begin(); it != obss.end(); it++)
        {
          cv::Point2f pix = it->second;
          cv::circle(img_debug, pix, 2, cv::Scalar(0, 255, 0), -1, 8);  // 绿色圆圈
        }

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img_debug;
        odom_viewer_.PublishOldAndNewAddedPointsInCurImg(out_msg.toImageMsg());
      }
    }

    /// [新增] 用于Gaussian-LIC的数据发布
    if (process_image && if_3dgs_)
    {
      Publish3DGSMappingData(msg);  // 发布3D高斯溅射建图数据
    }

    /// [8] 在RViz中可视化坐标变换
    auto pose = trajectory_->GetLidarPoseNURBS(msg.lidar_timestamp);      // 激光雷达位姿
    auto pose_debug = trajectory_->GetCameraPoseNURBS(msg.lidar_timestamp); // 相机位姿（调试用）
    // 发布激光雷达到地图的坐标变换
    odom_viewer_.PublishTF(pose.unit_quaternion(), pose.translation(), "lidar", "map");
    // 发布相机到地图的坐标变换
    odom_viewer_.PublishTF(pose_debug.unit_quaternion(), pose_debug.translation(), "camera", "map");
    // 发布地图到全局坐标系的坐标变换
    odom_viewer_.PublishTF(trajectory_manager_->GetGlobalFrame(), Eigen::Vector3d::Zero(), "map", "global");
  }

  // 准备两个时间段的消息数据
  bool OdometryManager::PrepareTwoSegMsgs(int seg_idx)
  {
    // 如果系统尚未初始化，返回false
    if (!is_initialized_)
      return false;

    // 获取轨迹数据的开始时间
    int64_t data_start_time = trajectory_->GetDataStartTime();
    
    // 处理激光雷达数据缓存：将时间戳转换为相对于轨迹开始时间的相对时间
    for (auto &data : msg_manager_->lidar_buf_)
    {
      if (!data.is_time_wrt_traj_start)  // 如果时间戳不是相对于轨迹开始时间
      {
        data.ToRelativeMeasureTime(data_start_time);  // 转换为相对时间
        msg_manager_->lidar_max_timestamps_[data.lidar_id] = data.max_timestamp;  // 更新该激光雷达的最大时间戳
      }
    }
    
    // 处理图像数据缓存：将时间戳转换为相对于轨迹开始时间的相对时间
    for (auto &data : msg_manager_->image_buf_)
    {
      if (!data.is_time_wrt_traj_start)  // 如果时间戳不是相对于轨迹开始时间
      {
        data.ToRelativeMeasureTime(data_start_time);  // 转换为相对时间
        msg_manager_->image_max_timestamp_ = data.timestamp;  // 更新图像最大时间戳
      }
    }
    
    // 移除时间过早的数据（在轨迹开始时间之前的数据）
    msg_manager_->RemoveBeginData(data_start_time, 0);

    // 计算当前时间段的结束时间
    // 第一段：轨迹当前最大时间 + t_add_ns_
    // 第二段：轨迹当前最大时间 + 2*t_add_ns_
    int64_t traj_max_time_ns = trajectory_->maxTimeNsNURBS() + t_add_ns_ * (seg_idx + 1);

    // 根据段索引获取对应时间区间的消息数据
    bool have_msg = false;
    if (seg_idx == 0)  // 准备第一段数据
    {
      int64_t traj_last_max_time_ns = trajectory_->maxTimeNsNURBS();  // 上一个时间段的结束时间
      // 获取从上一个时间段结束到当前时间段结束的消息
      have_msg = msg_manager_->GetMsgs(msg_manager_->cur_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);
    }
    if (seg_idx == 1)  // 准备第二段数据
    {
      int64_t traj_last_max_time_ns = trajectory_->maxTimeNsNURBS() + t_add_ns_;  // 第一段的结束时间
      // 获取从第一段结束到第二段结束的消息
      have_msg = msg_manager_->GetMsgs(msg_manager_->next_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);
    }

    // 如果成功获取到消息数据
    if (have_msg)
    {
      // 将所有IMU数据添加到轨迹管理器中
      while (!msg_manager_->imu_buf_.empty())
      {
        trajectory_manager_->AddIMUData(msg_manager_->imu_buf_.front());  // 添加IMU数据
        msg_manager_->imu_buf_.pop_front();  // 移除已处理的数据
      }
      
      // 根据段索引更新对应的时间戳
      if (seg_idx == 0)
      {
        traj_max_time_ns_cur = traj_max_time_ns;  // 更新当前段的最大时间
      }
      if (seg_idx == 1)
      {
        traj_max_time_ns_next = traj_max_time_ns;  // 更新下一段的最大时间
      }
      return true;  // 成功准备数据
    }
    else
    {
      return false;  // 未能获取到足够的数据
    }
  }

  // 更新两个轨迹段：根据IMU数据决定B样条控制点密度并添加节点
  void OdometryManager::UpdateTwoSeg()
  {
    // 获取所有IMU数据
    auto imu_datas = trajectory_manager_->GetIMUData();

    /// 更新第一段轨迹
    {
      int cp_add_num = cp_add_num_coarse_;  // 初始化控制点数量为粗分割数量
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();  // 角速度和加速度平均值
      double var_r = 0, var_a = 0;  // 角速度和加速度方差
      int cnt = 0;  // 统计有效IMU数据的数量
      
      // 计算第一段时间区间内IMU数据的平均值
      for (int i = 0; i < imu_datas.size(); i++)
      {
        // 筛选第一段时间区间内的IMU数据
        if (imu_datas[i].timestamp < trajectory_->maxTimeNsNURBS() ||
            imu_datas[i].timestamp >= traj_max_time_ns_cur)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;   // 累加角速度
        aver_a += imu_datas[i].accel;  // 累加加速度
      }
      aver_r /= cnt;  // 计算角速度平均值
      aver_a /= cnt;  // 计算加速度平均值
      
      // 计算第一段时间区间内IMU数据的方差
      for (int i = 0; i < imu_datas.size(); i++)
      {
        // 筛选第一段时间区间内的IMU数据
        if (imu_datas[i].timestamp < trajectory_->maxTimeNURBS() ||
            imu_datas[i].timestamp >= traj_max_time_ns_cur)
          continue;
        // 计算方差：每个数据与平均值的差的平方和
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));  // 计算角速度标准差
      var_a = sqrt(var_a / (cnt - 1));  // 计算加速度标准差
      // LOG(INFO) << "[aver_r_first] " << aver_r.norm() << " | [aver_a_first] " << aver_a.norm();
      // LOG(INFO) << "[var_r_first] " << var_r << " | [var_a_first] " << var_a;

      // 如果使用非均匀B样条，根据IMU运动强度调整控制点数量
      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());  // 根据角速度和加速度模长决定节点密度
      }
      // LOG(INFO) << "[cp_add_num_first] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);  // 记录控制点数量

      // 计算时间步长：将时间区间均匀分割为cp_add_num段
      int64_t step = (traj_max_time_ns_cur - trajectory_->maxTimeNsNURBS()) / cp_add_num;
      // LOG(INFO) << "[extend_step_first] " << step;
      
      // 添加中间节点（除了最后一个节点）
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = trajectory_->maxTimeNsNURBS() + step * (i + 1);
        trajectory_->AddKntNs(time);  // 添加B样条节点
      }
      trajectory_->AddKntNs(traj_max_time_ns_cur);  // 添加第一段的结束时间节点

      cp_add_num_cur = cp_add_num;  // 保存当前段的控制点数量
    }

    /// 更新第二段轨迹
    {
      int cp_add_num = cp_add_num_coarse_;  // 初始化控制点数量为粗分割数量
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();  // 角速度和加速度平均值
      double var_r = 0, var_a = 0;  // 角速度和加速度方差
      int cnt = 0;  // 统计有效IMU数据的数量
      
      // 计算第二段时间区间内IMU数据的平均值
      for (int i = 0; i < imu_datas.size(); i++)
      {
        // 筛选第二段时间区间内的IMU数据
        if (imu_datas[i].timestamp < traj_max_time_ns_cur ||
            imu_datas[i].timestamp >= traj_max_time_ns_next)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;   // 累加角速度
        aver_a += imu_datas[i].accel;  // 累加加速度
      }
      aver_r /= cnt;  // 计算角速度平均值
      aver_a /= cnt;  // 计算加速度平均值
      
      // 计算第二段时间区间内IMU数据的方差
      for (int i = 0; i < imu_datas.size(); i++)
      {
        // 筛选第二段时间区间内的IMU数据
        if (imu_datas[i].timestamp < traj_max_time_ns_cur ||
            imu_datas[i].timestamp >= traj_max_time_ns_next)
          continue;
        // 计算方差：每个数据与平均值的差的平方和
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));  // 计算角速度标准差
      var_a = sqrt(var_a / (cnt - 1));  // 计算加速度标准差
      // LOG(INFO) << "[aver_r_second] " << aver_r.norm() << " | [aver_a_second] " << aver_a.norm();
      // LOG(INFO) << "[var_r_second] " << var_r << " | [var_a_second] " << var_a;

      // 如果使用非均匀B样条，根据IMU运动强度调整控制点数量
      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());  // 根据角速度和加速度模长决定节点密度
      }
      // LOG(INFO) << "[cp_add_num_second] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);  // 记录控制点数量

      // 计算时间步长：将时间区间均匀分割为cp_add_num段
      int64_t step = (traj_max_time_ns_next - traj_max_time_ns_cur) / cp_add_num;
      // LOG(INFO) << "[extend_step_second] " << step;
      
      // 添加中间节点（除了最后一个节点）
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = traj_max_time_ns_cur + step * (i + 1);
        trajectory_->AddKntNs(time);  // 添加B样条节点
      }
      trajectory_->AddKntNs(traj_max_time_ns_next);  // 添加第二段的结束时间节点

      cp_add_num_next = cp_add_num;  // 保存下一段的控制点数量
    }
  }

  // 为下一个时间段准备消息数据（滑动窗口中的第三段）
  bool OdometryManager::PrepareMsgs()
  {
    // 如果系统尚未初始化，返回false
    if (!is_initialized_)
      return false;

    // 获取轨迹数据的开始时间
    int64_t data_start_time = trajectory_->GetDataStartTime();
    
    // 处理激光雷达数据缓存：将时间戳转换为相对于轨迹开始时间的相对时间
    for (auto &data : msg_manager_->lidar_buf_)
    {
      if (!data.is_time_wrt_traj_start)  // 如果时间戳不是相对于轨迹开始时间
      {
        data.ToRelativeMeasureTime(data_start_time);  // 转换为相对时间
        msg_manager_->lidar_max_timestamps_[data.lidar_id] = data.max_timestamp;  // 更新该激光雷达的最大时间戳
      }
    }
    
    // 处理图像数据缓存：将时间戳转换为相对于轨迹开始时间的相对时间
    for (auto &data : msg_manager_->image_buf_)
    {
      if (!data.is_time_wrt_traj_start)  // 如果时间戳不是相对于轨迹开始时间
      {
        data.ToRelativeMeasureTime(data_start_time);  // 转换为相对时间
        msg_manager_->image_max_timestamp_ = data.timestamp;  // 更新图像最大时间戳
      }
    }
    
    // 移除时间过早的数据（在轨迹开始时间之前的数据）
    msg_manager_->RemoveBeginData(data_start_time, 0);

    // 计算新时间段的结束时间：下一段的结束时间 + 时间增量
    int64_t traj_max_time_ns = traj_max_time_ns_next + t_add_ns_;

    // 设置新时间段的起始时间：等于当前下一段的结束时间
    int64_t traj_last_max_time_ns = traj_max_time_ns_next;
    
    // 获取新时间段内的消息数据（存储到next_next_msgs中）
    bool have_msg = msg_manager_->GetMsgs(msg_manager_->next_next_msgs, 
                                        traj_last_max_time_ns, 
                                        traj_max_time_ns, 
                                        data_start_time);

    // 如果成功获取到消息数据
    if (have_msg)
    {
      // 将所有IMU数据添加到轨迹管理器中
      while (!msg_manager_->imu_buf_.empty())
      {
        trajectory_manager_->AddIMUData(msg_manager_->imu_buf_.front());  // 添加IMU数据
        msg_manager_->imu_buf_.pop_front();  // 移除已处理的数据
      }
      
      // 更新下下个时间段的最大时间戳
      traj_max_time_ns_next_next = traj_max_time_ns;
      return true;  // 成功准备数据
    }
    else
    {
      return false;  // 未能获取到足够的数据
    }
  }

  // 更新单个轨迹段：为新的时间段添加B样条节点
  void OdometryManager::UpdateOneSeg()
  {
    // 获取所有IMU数据
    auto imu_datas = trajectory_manager_->GetIMUData();

    /// 更新新的时间段（滑动窗口中的第三段）
    {
      int cp_add_num = cp_add_num_coarse_;  // 初始化控制点数量为粗分割数量
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();  // 角速度和加速度平均值
      double var_r = 0, var_a = 0;  // 角速度和加速度方差
      int cnt = 0;  // 统计有效IMU数据的数量
      
      // 计算新时间段内IMU数据的平均值
      for (int i = 0; i < imu_datas.size(); i++)
      {
        // 筛选新时间段内的IMU数据：[traj_max_time_ns_next, traj_max_time_ns_next_next)
        if (imu_datas[i].timestamp < traj_max_time_ns_next ||
            imu_datas[i].timestamp >= traj_max_time_ns_next_next)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;   // 累加角速度
        aver_a += imu_datas[i].accel;  // 累加加速度
      }
      aver_r /= cnt;  // 计算角速度平均值
      aver_a /= cnt;  // 计算加速度平均值
      
      // 计算新时间段内IMU数据的方差
      for (int i = 0; i < imu_datas.size(); i++)
      {
        // 筛选新时间段内的IMU数据
        if (imu_datas[i].timestamp < traj_max_time_ns_next ||
            imu_datas[i].timestamp >= traj_max_time_ns_next_next)
          continue;
        // 计算方差：每个数据与平均值的差的平方和
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));  // 计算角速度标准差
      var_a = sqrt(var_a / (cnt - 1));  // 计算加速度标准差
      // LOG(INFO) << "[aver_r_new] " << aver_r.norm() << " | [aver_a_new] " << aver_a.norm();
      // LOG(INFO) << "[var_r_new] " << var_r << " | [var_a_new] " << var_a;

      // 如果使用非均匀B样条，根据IMU运动强度调整控制点数量
      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());  // 根据角速度和加速度模长决定节点密度
      }
      // LOG(INFO) << "[cp_add_num_new] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);  // 记录控制点数量到历史向量中

      // 计算时间步长：将新时间段均匀分割为cp_add_num段
      int64_t step = (traj_max_time_ns_next_next - traj_max_time_ns_next) / cp_add_num;
      // LOG(INFO) << "[extend_step_new] " << step;
      
      // 添加中间节点（除了最后一个节点）
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = traj_max_time_ns_next + step * (i + 1);  // 计算中间节点的时间
        trajectory_->AddKntNs(time);  // 添加B样条节点到轨迹中
      }
      trajectory_->AddKntNs(traj_max_time_ns_next_next);  // 添加新时间段的结束时间节点

      cp_add_num_next_next = cp_add_num;  // 保存新时间段的控制点数量
    }
  }

  // 设置系统初始状态
  void OdometryManager::SetInitialState()
  {
    // 检查系统是否已经初始化过
    if (is_initialized_)
    {
      // 断言检查轨迹数据开始时间是否有效
      assert(trajectory_->GetDataStartTime() > 0 && "data start time < 0");
      // 输出错误信息：系统状态已经被初始化
      std::cout << RED << "[Error] system state has been initialized" << RESET << std::endl;
      return;  // 直接返回，避免重复初始化
    }

    // 设置初始化标志为true，表示系统已完成初始化
    is_initialized_ = true;

    // 如果IMU初始化器已完成初始化过程
    if (imu_initializer_->InitialDone())
    {
      // 获取IMU状态（I0toG：初始IMU坐标系到全局坐标系的变换）
      SystemState sys_state = imu_initializer_->GetIMUState();
      
      // 将系统状态设置到轨迹管理器中，同时传入初始距离参数
      trajectory_manager_->SetSystemState(sys_state, distance0_);

      // 添加IMU初始化器中的最后一个IMU数据到轨迹管理器
      trajectory_manager_->AddIMUData(imu_initializer_->GetIMUData().back());
      
      // 清空消息管理器中的IMU数据缓冲区（避免重复处理）
      msg_manager_->imu_buf_.clear();
    }
    
    // 最终检查：确保轨迹数据开始时间有效
    assert(trajectory_->GetDataStartTime() > 0 && "data start time < 0");
  }

  void OdometryManager::PublishCloudAndTrajectory()
  {
    odom_viewer_.PublishDenseCloud(trajectory_, lidar_handler_->GetFeatureMapDs(),
                                   lidar_handler_->GetFeatureCurrent());

    odom_viewer_.PublishSplineTrajectory(
        trajectory_, 0.0, trajectory_->maxTimeNURBS(), 0.1);
  }

  // 发布3D高斯溅射建图数据：为3DGS算法提供图像、位姿和彩色点云数据
  void OdometryManager::Publish3DGSMappingData(const NextMsgs& msg)
  {
    // 将当前数据加入缓冲队列
    time_buf.push(msg.image_timestamp);                    // 图像时间戳
    lidar_buf.push(lidar_handler_->GetFeatureCurrent());   // 激光雷达特征数据
    img_buf.push(camera_handler_->img_pose_->m_img);       // 图像数据

    // 处理已稳定的数据（避免处理正在优化的数据）
    while(1)
    {
      int64_t active_time = trajectory_->GetActiveTime();  // 获取轨迹的活跃时间边界
      
      // 检查队列中最早的数据是否已稳定（时间小于活跃时间）
      if (time_buf.front() < active_time && lidar_buf.front().time_max < active_time)
      {
        // 取出队列中最早的数据进行处理
        auto time = time_buf.front();                      // 图像时间戳
        auto lidar = lidar_buf.front();                    // 激光雷达数据
        auto img = img_buf.front();                        // 图像数据
        time_buf.pop();                                    // 从队列中移除
        lidar_buf.pop();
        img_buf.pop();

        // 对激光雷达点云进行去畸变处理
        PosCloud::Ptr cloud_undistort_ds = PosCloud::Ptr(new PosCloud);  // 去畸变后的点云
        // PosCloud::Ptr cloud_distort_ds = lidar.surface_features;     // 可选：使用表面特征
        PosCloud::Ptr cloud_distort_ds = lidar.full_cloud;              // 使用完整点云
        if (cloud_distort_ds->size() != 0)
        {
          // 在全局坐标系下去除点云运动畸变
          trajectory_->UndistortScanInG(*cloud_distort_ds, lidar.timestamp, *cloud_undistort_ds);
        }

        // 发布图像数据：转换为绝对时间戳
        odom_viewer_.Publish3DGSImage(img, time + trajectory_->GetDataStartTime());

        // 获取相机位姿信息
        auto pose_cam = trajectory_->GetCameraPoseNURBS(time);        // 相机在世界坐标系下的位姿
        auto inv_pose_cam = pose_cam.inverse();                      // 逆变换：世界到相机
        auto cam_K = camera_handler_->m_camera_intrinsic;            // 相机内参矩阵
        double fx = cam_K(0, 0), fy = cam_K(1, 1);                  // 焦距
        double cx = cam_K(0, 2), cy = cam_K(1, 2);                  // 主点坐标
        int H = camera_handler_->img_pose_->m_img.rows;              // 图像高度
        int W = camera_handler_->img_pose_->m_img.cols;              // 图像宽度

        // 发布相机位姿：转换为绝对时间戳
        odom_viewer_.Publish3DGSPose(pose_cam.unit_quaternion(), pose_cam.translation(), 
                                    time + trajectory_->GetDataStartTime());

        // 处理点云：投影到图像获取颜色信息
        int filter_cnt = 0;                                         // 过滤掉的点数统计
        int skip = lidar_skip_;                                      // 点云采样间隔
        Eigen::aligned_vector<Eigen::Vector3d> new_points;          // 输出：3D点坐标
        Eigen::aligned_vector<Eigen::Vector3i> new_colors;          // 输出：RGB颜色值
        
        // 遍历去畸变后的点云（按采样间隔）
        for (int i = 0; i < cloud_undistort_ds->points.size(); i += skip)
        {
          auto pt = cloud_undistort_ds->points[i];
          Eigen::Vector3d pt_w = Eigen::Vector3d(pt.x, pt.y, pt.z);  // 世界坐标系下的点
          
          // 将3D点从世界坐标系变换到相机坐标系
          Eigen::Vector3d pt_c = inv_pose_cam.unit_quaternion().toRotationMatrix() * pt_w + inv_pose_cam.translation();
          
          // 过滤掉相机后方的点
          if (pt_c(2) < 0.01) 
          {
            filter_cnt++;
            continue;
          }
          
          // 归一化到图像平面
          pt_c /= pt_c(2);
          
          // 投影到像素坐标
          double u = fx * pt_c(0) + cx;
          double v = fy * pt_c(1) + cy;
          
          // 过滤掉图像边界外的点
          if (u < 0 || u > W - 1) 
          {
            filter_cnt++;
            continue;
          }
          
          // 添加有效的3D点
          new_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));

          // 获取投影点的颜色：使用双线性插值
          int i_u = std::round(u), i_v = std::round(v);             // 最近邻像素坐标
          int blue = 0, green = 0, red = 0;                         // RGB颜色分量
          
          if (i_u >= 0 && i_u < W && i_v >= 0 && i_v < H)           // 确保在图像范围内
          {
            // 双线性插值的四个邻近像素
            int u0 = std::floor(u), v0 = std::floor(v);
            int u1 = std::min(u0 + 1, W - 1), v1 = std::min(v0 + 1, H - 1);
            double du = u - u0, dv = v - v0;                        // 插值权重

            // 获取四个邻近像素的颜色值
            cv::Vec3b c00 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v0, u0);
            cv::Vec3b c10 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v0, u1);
            cv::Vec3b c01 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v1, u0);
            cv::Vec3b c11 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v1, u1);

            // 转换为Eigen向量
            Eigen::Vector3d color00(c00[0], c00[1], c00[2]);
            Eigen::Vector3d color10(c10[0], c10[1], c10[2]);
            Eigen::Vector3d color01(c01[0], c01[1], c01[2]);
            Eigen::Vector3d color11(c11[0], c11[1], c11[2]);

            // 双线性插值计算最终颜色
            Eigen::Vector3d interpolated_color = 
                (1 - du) * (1 - dv) * color00 +     // 左上
                du * (1 - dv) * color10 +           // 右上
                (1 - du) * dv * color01 +           // 左下
                du * dv * color11;                  // 右下
            
            // 转换为整数颜色值
            blue = std::round(interpolated_color.x());
            green = std::round(interpolated_color.y());
            red = std::round(interpolated_color.z());
          }
          // 添加颜色信息（RGB格式）
          new_colors.push_back(Eigen::Vector3i(red, green, blue));
        }
        
        // 发布带颜色的3D点云数据：转换为绝对时间戳
        odom_viewer_.Publish3DGSPoints(new_points, new_colors, time + trajectory_->GetDataStartTime());
      }
      else break;  // 如果没有更多稳定数据，退出循环
    }
  }

  double OdometryManager::SaveOdometry()
  {
    std::string descri;
    if (odometry_mode_ == LICO)
      descri = "LICO";
    else if (odometry_mode_ == LIO)
      descri = "LIO";

    if (msg_manager_->NumLiDAR() > 1)
      descri = descri + "2";

    ros::Time timer;
    std::string time_full_str = std::to_string(timer.now().toNSec());
    std::string t_str = "_" + time_full_str.substr(time_full_str.size() - 4);

    int idx = -1;
    int64_t true_maxtime = trajectory_->maxTimeNsNURBS();
    for (int i = trajectory_->knts.size() - 1; i >= 0; i--)
    {
      if (true_maxtime == trajectory_->knts[i])
      {
        idx = i;
        break;
      }
    }
    idx -= 1;
    int64_t maxtime = trajectory_->knts[idx];
    maxtime = trajectory_->maxTimeNsNURBS() - 0.1 * S_TO_NS;

    trajectory_->ToTUMTxt(cache_path_ + "_" + descri + ".txt", maxtime, is_evo_viral_,
                          0.01);  // 100Hz pose querying

    // int sum_cp = std::accumulate(cp_num_vec.begin(), cp_num_vec.end(), 0);
    // std::cout << GREEN << "ave_cp_num " << sum_cp * 1.0 / cp_num_vec.size() << RESET << std::endl;

    return trajectory_->maxTimeNURBS();
  }

} // namespace cocolic
