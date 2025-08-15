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

#include <odom/msg_manager.h>
#include <utils/parameter_struct.h>

#include <pcl/common/transforms.h>

namespace cocolic
{

  MsgManager::MsgManager(const YAML::Node &node, const std::string &config_path, ros::NodeHandle &nh)
      : has_valid_msg_(true),
        t_offset_imu_(0),
        t_offset_camera_(0),
        cur_imu_timestamp_(-1),
        cur_pose_timestamp_(-1),
        use_image_(false),
        lidar_timestamp_end_(false),
        remove_wrong_time_imu_(false),
        if_normalized_(false),
        image_topic_(""),
        use_subscriber_mode_(false)
  {
    OdometryMode odom_mode = OdometryMode(node["odometry_mode"].as<int>());

    nh.param<std::string>("bag_path", bag_path_, "");
    if (bag_path_ == "")
    {
      bag_path_ = node["bag_path"].as<std::string>();
    }

    /// imu topic
    std::string imu_yaml = node["imu_yaml"].as<std::string>();
    YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);
    imu_topic_ = imu_node["imu_topic"].as<std::string>();
    // pose_topic_ = imu_node["pose_topic"].as<std::string>();
    // remove_wrong_time_imu_ = imu_node["remove_wrong_time_imu"].as<bool>();
    if_normalized_ = imu_node["if_normalized"].as<bool>();

    // double imu_frequency = node["imu_frequency"].as<double>();
    // double imu_period_s = 1. / imu_frequency;

    std::string cam_yaml = node["camera_yaml"].as<std::string>();
    YAML::Node cam_node = YAML::LoadFile(config_path + cam_yaml);

    // add_extra_timeoffset_s_ =
    //     yaml::GetValue<double>(node, "add_extra_timeoffset_s", 0);
    // LOG(INFO) << "add_extra_timeoffset_s: " << add_extra_timeoffset_s_;
    // std::cout << "add_extra_timeoffset_s: " << add_extra_timeoffset_s_ << "\n";

    /// image topic
    if (odom_mode == OdometryMode::LICO)
      use_image_ = true;
    if (use_image_)
    {
      std::string cam_yaml = config_path + node["camera_yaml"].as<std::string>();
      YAML::Node cam_node = YAML::LoadFile(cam_yaml);
      image_topic_ = cam_node["image_topic"].as<std::string>();
      image_topic_compressed_ = std::string(image_topic_).append("/compressed");

      pub_img_ = nh.advertise<sensor_msgs::Image>("/vio/test_img", 1000);
    }
    image_max_timestamp_ = -1;

    /// lidar topic
    std::string lidar_yaml = node["lidar_yaml"].as<std::string>();
    YAML::Node lidar_node = YAML::LoadFile(config_path + lidar_yaml);
    num_lidars_ = lidar_node["num_lidars"].as<int>();
    lidar_timestamp_end_ = lidar_node["lidar_timestamp_end"].as<bool>();

    bool use_livox = false;
    bool use_vlp = false;
    bool use_hesai = false;
    for (int i = 0; i < num_lidars_; ++i)
    {
      std::string lidar_str = "lidar" + std::to_string(i);
      const auto &lidar_i = lidar_node[lidar_str];
      bool is_livox = lidar_i["is_livox"].as<bool>();
      bool is_hesai = lidar_i["is_hesai"].as<bool>();
      if (is_livox)
      {
        lidar_types.push_back(LIVOX);
        use_livox = true;
      }
      else if (is_hesai)
      {
        lidar_types.push_back(HESAI);
        use_hesai = true;
      }
      else
      {
        lidar_types.push_back(VLP);
        use_vlp = true;
      }
      lidar_topics_.push_back(lidar_i["topic"].as<std::string>());
      EP_LktoI_.emplace_back();
      EP_LktoI_.back().Init(lidar_i["Extrinsics"]);
    }

    for (int k = 0; k < num_lidars_; ++k)
    {
      lidar_max_timestamps_.push_back(0);
      Eigen::Matrix4d T_Lk_to_L0 = Eigen::Matrix4d::Identity();
      if (k > 0)
      {
        T_Lk_to_L0.block<3, 3>(0, 0) =
            (EP_LktoI_[0].q.inverse() * EP_LktoI_[k].q).toRotationMatrix();
        T_Lk_to_L0.block<3, 1>(0, 3) =
            EP_LktoI_[0].q.inverse() * (EP_LktoI_[k].p - EP_LktoI_[0].p);

        // std::cout << "lidar " << k << "\n"
        //           << T_Lk_to_L0 << std::endl;
      }
      T_LktoL0_vec_.push_back(T_Lk_to_L0);
    }

    if (use_livox)
      livox_feature_extraction_ =
          std::make_shared<LivoxFeatureExtraction>(lidar_node);
    if (use_vlp)
      velodyne_feature_extraction_ =
          std::make_shared<VelodyneFeatureExtraction>(lidar_node);
    if (use_hesai)
      velodyne_feature_extraction_ =
          std::make_shared<VelodyneFeatureExtraction>(lidar_node);

    // 读取模式配置
    use_subscriber_mode_ = node["use_subscriber_mode"].as<bool>(false);
    
    if (use_subscriber_mode_) {
      // 订阅者模式：初始化ROS订阅者
      std::cout << "\n🔄 使用订阅者模式 (rostopic)\n";
      InitializeSubscribers(nh);
    } else {
      // 原有bag文件模式
      std::cout << "\n📁 使用 Bag 文件模式\n";
      LoadBag(node);
    }
  }

  /**
   * [功能描述]：初始化ROS订阅者，用于接收实时数据流
   * @param nh：ROS节点句柄引用
   */
  void MsgManager::InitializeSubscribers(ros::NodeHandle &nh)
  {
    // 初始化IMU订阅者
    sub_imu_ = nh.subscribe<sensor_msgs::Imu>(
        imu_topic_, 1000, 
        boost::bind(&MsgManager::IMUCallback, this, _1));

    // 初始化雷达订阅者
    subs_vlp16_.clear();
    subs_livox_.clear();
    
    for (int i = 0; i < num_lidars_; ++i) {
      if (lidar_types[i] == VLP) {
        // Velodyne雷达订阅者
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topics_[i], 10,
            boost::bind(&MsgManager::VelodyneCallback, this, _1, i));
        subs_vlp16_.push_back(sub);
      } 
      else if (lidar_types[i] == LIVOX) {
        // Livox雷达订阅者
        ros::Subscriber sub = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topics_[i], 10,
            boost::bind(&MsgManager::LivoxCallback, this, _1, i));
        subs_livox_.push_back(sub);
      }
      else if (lidar_types[i] == HESAI) {
        // Hesai雷达订阅者
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topics_[i], 10,
            boost::bind(&MsgManager::HesaiCallback, this, _1, i));
        ros::Subscriber sub_angle = nh.subscribe<motor::MultiMotor>(
            "/multi_motor", 100000,
            boost::bind(&MsgManager::MotorCallback, this, _1));
        subs_hesai_.push_back(sub);
        subs_angle_.push_back(sub_angle);
        std::cout << "Hesai雷达订阅者初始化成功" << std::endl;
      }
    }

    // 初始化图像订阅者（如果需要）
    if (use_image_) {
      sub_image_ = nh.subscribe<sensor_msgs::Image>(
          image_topic_, 10,
          boost::bind(&MsgManager::ImageCallback, this, _1));
    }

    std::cout << "✅ ROS Subscribers initialized successfully!\n";
  }

  /**
   * [功能描述]：IMU订阅者回调函数，调用现有的IMU消息处理逻辑
   */
  void MsgManager::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    IMUMsgHandle(msg);  // 直接调用现有处理函数
  }

  /**
   * [功能描述]：Velodyne雷达订阅者回调函数
   */
  void MsgManager::VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int lidar_id)
  {
    CheckLidarMsgTimestamp(msg->header.stamp.toSec(), msg->header.stamp.toSec());
    VelodyneMsgHandle(msg, lidar_id);  // 调用现有处理函数
  }

  /**
   * [功能描述]：Livox雷达订阅者回调函数
   */
  void MsgManager::LivoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg, int lidar_id)
  {
    CheckLidarMsgTimestamp(msg->header.stamp.toSec(), msg->header.stamp.toSec());
    LivoxMsgHandle(msg, lidar_id);  // 调用现有处理函数
  }

  void MsgManager::MotorCallback(const motor::MultiMotor::ConstPtr &msg_in)
  {
    for (int i = 0; i < msg_in->motor_frames.size(); i++)
    {
      nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry());
      msg->header = msg_in->motor_frames[i].header;
      msg->twist.twist.angular.x = msg_in->motor_frames[i].angle_encoder;
      double timestamp = msg->header.stamp.toSec();
      // mtx_motor_buffer.lock();

      if (timestamp < last_timestamp_motor)
      {
        ROS_WARN("motor loop back, clear buffer");

        if (abs(last_timestamp_motor - timestamp) < 0.02)
        {
          last_timestamp_motor = motor_buffer.back()->header.stamp.toSec();
          motor_buffer.pop_back(); // 移除尾部元素
        }

        // mtx_motor_buffer.unlock();
        return;
      }
      last_timestamp_motor = timestamp;
      motor_buffer.emplace_back(msg);
      // mtx_motor_buffer.unlock();
    }
  }

  void MsgManager::HesaiCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int lidar_id)
  {
    static int scan_num = 0;
    scan_num++;
    if (scan_num < 10)
    {
      return;
    }
    CheckLidarMsgTimestamp(msg->header.stamp.toSec(), msg->header.stamp.toSec());
    HesaiMotorHandle(msg, motor_buffer, lidar_id);
  }

  /**
   * [功能描述]：图像订阅者回调函数
   */
  void MsgManager::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    ImageMsgHandle(msg);  // 调用现有处理函数
  }

  /**
   * [功能描述]：压缩图像订阅者回调函数
   */
  void MsgManager::CompressedImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
  {
    ImageMsgHandle(msg);  // 调用现有处理函数
  }

  void MsgManager::LoadBag(const YAML::Node &node)
  {
    double bag_start = node["bag_start"].as<double>();
    double bag_durr = node["bag_durr"].as<double>();

    std::vector<std::string> topics;
    topics.push_back(imu_topic_); // imu
    if (use_image_)               // camera
    {
      topics.push_back(image_topic_);
      topics.push_back(image_topic_compressed_);
    }
    for (auto &v : lidar_topics_) // lidar
      topics.push_back(v);
    // topics.push_back(pose_topic_);

    bag_.open(bag_path_, rosbag::bagmode::Read);

    rosbag::View view_full;
    view_full.addQuery(bag_);
    ros::Time time_start = view_full.getBeginTime();
    time_start += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime()
                                           : time_start + ros::Duration(bag_durr);
    view_.addQuery(bag_, rosbag::TopicQuery(topics), time_start, time_finish);
    if (view_.size() == 0)
    {
      ROS_ERROR("No messages to play on specified topics.  Exiting.");
      ros::shutdown();
      return;
    }

    std::cout << "\n🍺 LoadBag " << bag_path_ << " start at " << bag_start
              << " with duration " << (time_finish - time_start).toSec() << ".\n";
    // LOG(INFO) << "LoadBag " << bag_path_ << " start at " << bag_start
    //           << " with duration " << (time_finish - time_start).toSec();
  }

  /**
   * [功能描述]：从ROS bag文件中逐一读取并处理传感器消息，实现多传感器数据的统一管理
   * 该函数每次调用处理一条消息，支持IMU、LiDAR（Velodyne/Livox）和相机消息的解析
   */
  void MsgManager::SpinBagOnce()
  {
    if (!use_subscriber_mode_)
    {
      // 静态迭代器，保持在多次函数调用间的状态，用于遍历bag文件中的所有消息
      static rosbag::View::iterator view_iterator = view_.begin();
      
      // 检查是否已经到达bag文件末尾
      if (view_iterator == view_.end())
      {
        // 设置标志位表示没有更多有效消息可处理
        has_valid_msg_ = false;
        // LOG(INFO) << "End of bag"; // 调试信息：bag文件已处理完毕
        return;
      }

      // 获取当前消息实例的引用
      const rosbag::MessageInstance &m = *view_iterator;
      // 提取消息的topic名称，用于判断消息类型
      std::string msg_topic = m.getTopic();
      // 获取消息的时间戳信息
      auto msg_time = m.getTime();

      // 处理IMU消息：判断当前消息是否为IMU数据
      if (msg_topic == imu_topic_)  // imu
      {
        // 将消息实例化为IMU消息类型的智能指针
        sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        // 调用IMU消息处理函数，进行数据解析和存储
        IMUMsgHandle(imu_msg);
      }
      // 处理LiDAR消息：检查消息topic是否在配置的雷达topic列表中
      else if (std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic) !=
              lidar_topics_.end())  // lidar
      {
        // 找到当前topic在雷达topic列表中的位置
        auto it = std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic);
        // 计算雷达索引，用于区分多个雷达传感器
        auto idx = std::distance(lidar_topics_.begin(), it);
        
        // 处理机械式雷达（如Velodyne、Ouster、Hesai等旋转雷达）
        if (lidar_types[idx] == VLP)  //[rotating lidar: Velodyne、Ouster、Hesai]
        {
          // 类型安全检查：确保消息类型为PointCloud2
          if (!m.isType<sensor_msgs::PointCloud2>())
            std::cout << "Wrong type\n";

          // 实例化为PointCloud2消息类型
          auto lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
          // 检查雷达消息时间戳的一致性，确保数据同步
          CheckLidarMsgTimestamp(msg_time.toSec(), lidar_msg->header.stamp.toSec());
          // 调用Velodyne雷达消息处理函数，进行特征提取和数据处理
          VelodyneMsgHandle(lidar_msg, idx);
          // VelodyneMsgHandleNoFeature(lidar_msg, idx); // 备选处理方式：不进行特征提取
        }
        // 处理固态雷达（Livox雷达系列）
        else if (lidar_types[idx] == LIVOX)  //[solid-state lidar: Livox]
        {
          // 类型安全检查：确保消息类型为Livox自定义消息格式
          if (!m.isType<livox_ros_driver::CustomMsg>())
            std::cout << "Wrong type\n";

          // 实例化为Livox自定义消息类型
          auto lidar_msg = m.instantiate<livox_ros_driver::CustomMsg>();
          // 检查雷达消息时间戳的一致性
          CheckLidarMsgTimestamp(msg_time.toSec(), lidar_msg->header.stamp.toSec());
          // 调用Livox雷达消息处理函数，处理固态雷达特有的数据格式
          LivoxMsgHandle(lidar_msg, idx);
        }
      }
      // 处理相机消息：判断是否为原始图像或压缩图像topic
      else if (msg_topic == image_topic_ || msg_topic == image_topic_compressed_)  // camera
      {
        // 处理压缩图像格式
        if (m.getDataType() == "sensor_msgs/CompressedImage")
        {
          // 实例化为压缩图像消息类型
          sensor_msgs::CompressedImageConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
          // 调用图像消息处理函数，进行图像解码和预处理
          ImageMsgHandle(image_msg);
        }
        // 处理原始图像格式
        else if (m.getDataType() == "sensor_msgs/Image")
        {
          // 实例化为原始图像消息类型
          sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
          // 调用图像消息处理函数
          ImageMsgHandle(image_msg);
        }
      }

      // 移动迭代器到下一条消息，为下次调用做准备
      view_iterator++;
    }
    else
    {
      // 订阅者模式：处理ROS订阅者回调函数
      ros::spinOnce();  // 处理回调函数
      usleep(10);     // 短暂延时，避免CPU占用过高
    }
  }

  void MsgManager::LogInfo() const
  {
    int m_size[3] = {0, 0, 0};
    m_size[0] = imu_buf_.size();
    m_size[1] = lidar_buf_.size();
    // if (use_image_) m_size[2] = feature_tracker_node_->NumImageMsg();
    // LOG(INFO) << "imu/lidar/image msg left: " << m_size[0] << "/" << m_size[1]
    //           << "/" << m_size[2];
  }

  void MsgManager::RemoveBeginData(int64_t start_time, // not used
                                   int64_t relative_start_time)
  { // 0
    for (auto iter = lidar_buf_.begin(); iter != lidar_buf_.end();)
    {
      if (iter->timestamp < relative_start_time)
      {
        if (iter->max_timestamp <= relative_start_time)
        { // [1]
          iter = lidar_buf_.erase(iter);
          continue;
        }
        else
        { // [2]
          // int64_t t_aft = relative_start_time + 1e-3;  //1e-3
          int64_t t_aft = relative_start_time;
          LiDARCloudData scan_bef, scan_aft;
          scan_aft.timestamp = t_aft;
          scan_aft.max_timestamp = iter->max_timestamp;
          pcl::FilterCloudByTimestamp(iter->raw_cloud, t_aft,
                                      scan_bef.raw_cloud,
                                      scan_aft.raw_cloud);
          pcl::FilterCloudByTimestamp(iter->surf_cloud, t_aft,
                                      scan_bef.surf_cloud,
                                      scan_aft.surf_cloud);
          pcl::FilterCloudByTimestamp(iter->corner_cloud, t_aft,
                                      scan_bef.corner_cloud,
                                      scan_aft.corner_cloud);

          iter->timestamp = t_aft;
          *iter->raw_cloud = *scan_aft.raw_cloud;
          *iter->surf_cloud = *scan_aft.surf_cloud;
          *iter->corner_cloud = *scan_aft.corner_cloud;
        }
      }

      iter++;
    }

    if (use_image_)
    {
      for (auto iter = image_buf_.begin(); iter != image_buf_.end();)
      {
        if (iter->timestamp < relative_start_time)
        {
          iter = image_buf_.erase(iter); //
          continue;
        }
        iter++;
      }
    }
  }

  bool MsgManager::HasEnvMsg() const
  {
    int env_msg = lidar_buf_.size();
    // if (cur_imu_timestamp_ < 0 && env_msg > 100)
    //   LOG(WARNING) << "No IMU data. CHECK imu topic" << imu_topic_;

    return env_msg > 0;
  }

  bool MsgManager::CheckMsgIsReady(double traj_max, double start_time,
                                   double knot_dt, bool in_scan_unit) const
  {
    double t_imu_wrt_start = cur_imu_timestamp_ - start_time;

    //
    if (t_imu_wrt_start < traj_max)
    {
      return false;
    }

    // 
    int64_t t_front_lidar = -1;
    // Count how many unique lidar streams
    std::vector<int> unique_lidar_ids;
    for (const auto &data : lidar_buf_)
    {
      if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                    data.lidar_id) != unique_lidar_ids.end())
        continue;
      unique_lidar_ids.push_back(data.lidar_id);

      // 
      t_front_lidar = std::max(t_front_lidar, data.max_timestamp);
    }

    // 
    if ((int)unique_lidar_ids.size() != num_lidars_)
      return false;

    // 
    int64_t t_back_lidar = lidar_max_timestamps_[0];
    for (auto t : lidar_max_timestamps_)
    {
      t_back_lidar = std::min(t_back_lidar, t);
    }

    //  
    if (in_scan_unit)
    {
      // 
      if (t_front_lidar > t_imu_wrt_start)
        return false;
    }
    else
    {
      // 
      if (t_back_lidar < traj_max)
        return false;
    }

    return true;
  }

  bool MsgManager::AddImageToMsg(NextMsgs &msgs, const ImageData &image,
                                 int64_t traj_max)
  {
    if (image.timestamp >= traj_max)
      return false;
    msgs.if_have_image = true; // important!
    msgs.image_timestamp = image.timestamp;
    msgs.image = image.image;
    // msgs.image = image.image.clone();
    return true;
  }

  bool MsgManager::AddToMsg(NextMsgs &msgs, std::deque<LiDARCloudData>::iterator scan,
                            int64_t traj_max)
  {
    bool add_entire_scan = false;
    // if (scan->timestamp > traj_max) return add_entire_scan;

    if (scan->max_timestamp < traj_max)
    { // 
      *msgs.lidar_raw_cloud += (*scan->raw_cloud);
      *msgs.lidar_surf_cloud += (*scan->surf_cloud);
      *msgs.lidar_corner_cloud += (*scan->corner_cloud);

      // 
      if (msgs.scan_num == 0)
      {
        // first scan
        msgs.lidar_timestamp = scan->timestamp;
        msgs.lidar_max_timestamp = scan->max_timestamp;
      }
      else
      {
        msgs.lidar_timestamp =
            std::min(msgs.lidar_timestamp, scan->timestamp);
        msgs.lidar_max_timestamp =
            std::max(msgs.lidar_max_timestamp, scan->max_timestamp);
      }

      add_entire_scan = true;
    }
    else
    { // 
      LiDARCloudData scan_bef, scan_aft;
      pcl::FilterCloudByTimestamp(scan->raw_cloud, traj_max, scan_bef.raw_cloud,
                                  scan_aft.raw_cloud);
      pcl::FilterCloudByTimestamp(scan->surf_cloud, traj_max, scan_bef.surf_cloud,
                                  scan_aft.surf_cloud);
      pcl::FilterCloudByTimestamp(scan->corner_cloud, traj_max,
                                  scan_bef.corner_cloud, scan_aft.corner_cloud);
      //
      scan_bef.timestamp = scan->timestamp;
      scan_bef.max_timestamp = traj_max - 1e-9 * S_TO_NS;
      scan_aft.timestamp = traj_max;
      scan_aft.max_timestamp = scan->max_timestamp;

      // 
      scan->timestamp = traj_max;
      // *scan.max_timestamp = ； // 
      *scan->raw_cloud = *scan_aft.raw_cloud;
      *scan->surf_cloud = *scan_aft.surf_cloud;
      *scan->corner_cloud = *scan_aft.corner_cloud;

      *msgs.lidar_raw_cloud += (*scan_bef.raw_cloud);
      *msgs.lidar_surf_cloud += (*scan_bef.surf_cloud);
      *msgs.lidar_corner_cloud += (*scan_bef.corner_cloud);

      // 
      if (msgs.scan_num == 0)
      {
        // first scan
        msgs.lidar_timestamp = scan_bef.timestamp;
        msgs.lidar_max_timestamp = scan_bef.max_timestamp;
      }
      else
      {
        msgs.lidar_timestamp =
            std::min(msgs.lidar_timestamp, scan_bef.timestamp);
        msgs.lidar_max_timestamp =
            std::max(msgs.lidar_max_timestamp, scan_bef.max_timestamp);
      }

      add_entire_scan = false;
    }

    // 
    msgs.scan_num++;

    return add_entire_scan;
  }

  /// 
  // 从缓冲区中获取指定时间区间内的传感器数据
  bool MsgManager::GetMsgs(NextMsgs &msgs, int64_t traj_last_max, int64_t traj_max, int64_t start_time)
  {
    msgs.Clear();  // 清空消息容器

    // 基本条件检查：IMU和激光雷达缓冲区不能为空
    if (imu_buf_.empty() || lidar_buf_.empty())
    {
      return false;
    }
    
    // 检查当前IMU时间戳是否足够覆盖目标时间区间
    if (cur_imu_timestamp_ - start_time < traj_max)
    {
      return false;
    }

    /// 1. 检查所有激光雷达是否都有足够的数据
    
    // 统计缓冲区中有多少个不同的激光雷达ID
    std::vector<int> unique_lidar_ids;
    for (const auto &data : lidar_buf_)
    {
      // 如果该激光雷达ID已经存在，跳过
      if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                    data.lidar_id) != unique_lidar_ids.end())
        continue;
      unique_lidar_ids.push_back(data.lidar_id);  // 添加新的激光雷达ID
    }
    
    // 检查是否所有激光雷达都有数据
    if (unique_lidar_ids.size() != num_lidars_)
    {
      return false;
    }
    
    // 检查每个激光雷达的最大时间戳是否都满足要求
    for (auto t : lidar_max_timestamps_)
    {
      if (t < traj_max)  // 如果某个激光雷达的最大时间戳小于目标时间
      {
        return false;
      }
    }
    
    // 如果使用图像，检查图像数据是否足够
    if (use_image_)
    {
      if (image_max_timestamp_ < traj_max)
      {
        return false;
      }
    }

    /// 2. 处理激光雷达数据：提取指定时间区间内的点云数据
    
    for (auto it = lidar_buf_.begin(); it != lidar_buf_.end();)
    {
      // 如果当前激光雷达数据的时间戳大于等于目标时间，跳过
      std::cout << "it->timestamp = " << it->timestamp << std::endl;
      std::cout << "traj_max = " << traj_max << std::endl;
      if (it->timestamp >= traj_max)
      {
        ++it;
        continue;
      }
      
      // 尝试将当前激光雷达数据添加到消息中
      bool add_entire_scan = AddToMsg(msgs, it, traj_max);
      
      if (add_entire_scan)  // 如果整个扫描都被添加了
      {
        it = lidar_buf_.erase(it);  // 从缓冲区中移除该数据
      }
      else  // 如果只添加了部分数据
      {
        ++it;  // 保留该数据，继续处理下一个
      }
    }
    // LOG(INFO) << "[msgs_scan_num] " << msgs.scan_num;

    /// 3. 处理图像数据：选择合适的图像帧
    
    if (use_image_)
    {
      /// 查找时间区间 [traj_last_max, traj_max) 内最后一个图像帧
      int img_idx = INT_MAX;
      for (int i = 0; i < image_buf_.size(); i++)
      {
        // 如果图像时间戳在目标时间区间内
        if (image_buf_[i].timestamp >= traj_last_max &&
            image_buf_[i].timestamp < traj_max)
        {
          img_idx = i;  // 记录符合条件的图像索引（会选择最后一个）
        }
        // 如果图像时间戳超过目标时间，停止搜索
        if (image_buf_[i].timestamp >= traj_max)
        {
          break;
        }
      }

      /// 注释掉的代码：选择第一个符合条件的图像帧
      // int img_idx = INT_MAX;
      // for (int i = 0; i < image_buf_.size(); i++)
      // {
      //   if (image_buf_[i].timestamp >= traj_last_max &&
      //       image_buf_[i].timestamp < traj_max)
      //   {
      //     img_idx = i;
      //     break;  // 这里是break，会选择第一个符合条件的
      //   }
      // }

      // 如果找到了合适的图像帧
      if (img_idx != INT_MAX)
      {
        AddImageToMsg(msgs, image_buf_[img_idx], traj_max);  // 添加图像到消息中
        // image_buf_.erase(image_buf_.begin() + img_idx);  // 注释掉：不立即删除图像
      }
      else
      {
        msgs.if_have_image = false;  // 标记没有找到合适的图像
        // std::cout << "[GetMsgs does not get a image]\n";
        // std::getchar();
      }
    }

    return true;  // 成功获取到消息数据
  }

  void MsgManager::IMUMsgHandle(const sensor_msgs::Imu::ConstPtr &imu_msg)
  {
    int64_t t_last = cur_imu_timestamp_;
    // cur_imu_timestamp_ = imu_msg->header.stamp.toSec() - add_extra_timeoffset_s_;
    cur_imu_timestamp_ = imu_msg->header.stamp.toSec() * S_TO_NS;

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*imu_msg));
    if (true)
    {
      // imu 角速度 加速度矫正
      msg->angular_velocity.x = imu_msg->angular_velocity.y;
      msg->angular_velocity.y = imu_msg->angular_velocity.x;
      msg->linear_acceleration.x = imu_msg->linear_acceleration.y;
      msg->linear_acceleration.y = imu_msg->linear_acceleration.x;
    }

    IMUData data;
    IMUMsgToIMUData(msg, data);

    /// problem
    // data.timestamp -= add_extra_timeoffset_s_;

    // for trajectory_manager
    imu_buf_.emplace_back(data);
  }

  /**
   * [功能描述]：处理Velodyne机械式雷达的标准PointCloud2消息，解析点云数据并提取几何特征
   * @param vlp16_msg：Velodyne雷达PointCloud2消息的常量智能指针，包含原始点云数据
   * @param lidar_id：雷达标识符，用于区分多个雷达传感器（0表示主雷达）
   */
  void MsgManager::VelodyneMsgHandle(
      const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id)
  {
    // 创建原始点云数据容器，存储RTPoint类型点（包含x,y,z,intensity,ring,time信息）
    RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
    // 使用Velodyne特征提取器解析PointCloud2消息，转换为标准PCL点云格式
    velodyne_feature_extraction_->ParsePointCloud(vlp16_msg, vlp_raw_cloud);

    // 多雷达系统坐标变换：如果不是主雷达（ID≠0），先将点云变换到主雷达坐标系
    // 注意：与Livox处理不同，这里在特征提取之前进行坐标变换
    if (lidar_id != 0)
      pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
                              T_LktoL0_vec_[lidar_id]);

    // 执行雷达数据处理和特征提取，包括去畸变、降采样、特征分类等操作
    velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

    // 在雷达缓冲队列末尾创建新的数据条目，避免不必要的拷贝操作
    lidar_buf_.emplace_back();
    // 设置雷达ID，用于多雷达系统中的数据管理
    lidar_buf_.back().lidar_id = lidar_id;
    
    // 根据数据集特性进行时间戳校正处理
    if (lidar_timestamp_end_)
    {
      // 针对KAIST、VIRAL等数据集：雷达时间戳表示扫描结束时间，需要减去扫描时间（约0.1003秒）
      // 将时间戳校正为扫描开始时间，确保与IMU等传感器的时间同步
      lidar_buf_.back().timestamp = (vlp16_msg->header.stamp.toSec() - 0.1003) * S_TO_NS;
    }
    else
    {
      // 针对LVI-SAM、LIO-SAM等数据集：雷达时间戳直接表示扫描开始时间
      // 直接转换时间戳为纳秒单位，无需额外校正
      lidar_buf_.back().timestamp = vlp16_msg->header.stamp.toSec() * S_TO_NS;
    }
    
    // 存储已经过坐标变换的原始点云数据
    lidar_buf_.back().raw_cloud = vlp_raw_cloud;
    // 获取表面特征点云，用于平面约束优化（墙面、地面等平坦区域）
    lidar_buf_.back().surf_cloud =
        velodyne_feature_extraction_->GetSurfaceFeature();
    // 获取角点特征点云，用于边缘约束优化（建筑物棱角、柱子等突出特征）
    lidar_buf_.back().corner_cloud =
        velodyne_feature_extraction_->GetCornerFeature();
  }

  void MsgManager::HesaiMotorHandle(
    const sensor_msgs::PointCloud2::ConstPtr &msg,
    const std::deque<nav_msgs::Odometry::ConstPtr> &angle_msgs,
    int lidar_id)
  {
    if (angle_msgs.empty())
    {
      return;
    }
    while (motor_buffer.size() > 0 && motor_buffer.front()->header.stamp.toSec() < msg->header.stamp.toSec() - 0.5)
    {
      motor_buffer.pop_front();
    }
    // 创建原始点云数据容器，存储RTPoint类型点（包含x,y,z,intensity,ring,time信息）
    RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
    // 使用Velodyne特征提取器解析PointCloud2消息，转换为标准PCL点云格式
    velodyne_feature_extraction_->ParseHesaiMotor(msg, angle_msgs, vlp_raw_cloud);

    // 多雷达系统坐标变换：如果不是主雷达（ID≠0），先将点云变换到主雷达坐标系
    // 注意：与Livox处理不同，这里在特征提取之前进行坐标变换
    if (lidar_id != 0)
      pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
                              T_LktoL0_vec_[lidar_id]);

    // 执行雷达数据处理和特征提取，包括去畸变、降采样、特征分类等操作
    velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

    // 在雷达缓冲队列末尾创建新的数据条目，避免不必要的拷贝操作
    lidar_buf_.emplace_back();
    // 设置雷达ID，用于多雷达系统中的数据管理
    lidar_buf_.back().lidar_id = lidar_id;
    
    // 根据数据集特性进行时间戳校正处理
    if (lidar_timestamp_end_)
    {
      // 针对KAIST、VIRAL等数据集：雷达时间戳表示扫描结束时间，需要减去扫描时间（约0.1003秒）
      // 将时间戳校正为扫描开始时间，确保与IMU等传感器的时间同步
      lidar_buf_.back().timestamp = (msg->header.stamp.toSec() - 0.1003) * S_TO_NS;
    }
    else
    {
      // 针对LVI-SAM、LIO-SAM等数据集：雷达时间戳直接表示扫描开始时间
      // 直接转换时间戳为纳秒单位，无需额外校正
      lidar_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
    }
    
    // 存储已经过坐标变换的原始点云数据
    lidar_buf_.back().raw_cloud = vlp_raw_cloud;
    // 获取表面特征点云，用于平面约束优化（墙面、地面等平坦区域）
    lidar_buf_.back().surf_cloud =
        velodyne_feature_extraction_->GetSurfaceFeature();
    // 获取角点特征点云，用于边缘约束优化（建筑物棱角、柱子等突出特征）
    lidar_buf_.back().corner_cloud =
        velodyne_feature_extraction_->GetCornerFeature();
  }

  void MsgManager::VelodyneMsgHandleNoFeature(
      const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id)
  {
    RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
    velodyne_feature_extraction_->ParsePointCloudNoFeature(vlp16_msg, vlp_raw_cloud); // 

    // // transform the input cloud to Lidar0 frame
    // if (lidar_id != 0)
    //   pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
    //                            T_LktoL0_vec_[lidar_id]);

    // //
    // velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

    lidar_buf_.emplace_back();
    lidar_buf_.back().lidar_id = lidar_id;
    if (lidar_timestamp_end_)
    {
      lidar_buf_.back().timestamp = (vlp16_msg->header.stamp.toSec() - 0.1003) * S_TO_NS; // kaist、viral
    }
    else
    {
      lidar_buf_.back().timestamp = vlp16_msg->header.stamp.toSec() * S_TO_NS; // lvi、lio
    }
    lidar_buf_.back().raw_cloud = vlp_raw_cloud;
    lidar_buf_.back().surf_cloud =
        velodyne_feature_extraction_->GetSurfaceFeature();
    lidar_buf_.back().corner_cloud =
        velodyne_feature_extraction_->GetCornerFeature();
  }

  /**
     * [功能描述]：处理Livox固态雷达的自定义消息格式，解析点云数据并提取几何特征
     * @param livox_msg：Livox雷达自定义消息的常量智能指针，包含原始点云数据
     * @param lidar_id：雷达标识符，用于区分多个雷达传感器（0表示主雷达）
     */
    void MsgManager::LivoxMsgHandle(
      const livox_ros_driver::CustomMsg::ConstPtr &livox_msg, int lidar_id)
  {
    // 创建原始点云数据容器，存储RTPoint类型点（包含x,y,z,intensity,ring,time信息）
    RTPointCloud::Ptr livox_raw_cloud(new RTPointCloud);
    
    // 注释掉的代码段：提供了多种点云解析方式的选择
    // livox_feature_extraction_->ParsePointCloud(livox_msg, livox_raw_cloud);        // 标准解析方式
    // livox_feature_extraction_->ParsePointCloudNoFeature(livox_msg, livox_raw_cloud); // 无特征提取解析
    
    // 使用R3LIVE风格的点云解析方法，将Livox自定义消息转换为标准点云格式
    livox_feature_extraction_->ParsePointCloudR3LIVE(livox_msg, livox_raw_cloud);

    // 创建雷达点云数据结构，用于存储完整的雷达帧信息
    LiDARCloudData data;
    // 设置雷达ID，用于多雷达系统中的数据管理和坐标变换
    data.lidar_id = lidar_id;
    // 提取消息时间戳并转换为纳秒单位，确保时间精度
    data.timestamp = livox_msg->header.stamp.toSec() * S_TO_NS;
    // 存储原始点云数据，包含所有未处理的雷达测量点
    data.raw_cloud = livox_raw_cloud;
    // 获取表面特征点云，用于平面约束优化
    data.surf_cloud = livox_feature_extraction_->GetSurfaceFeature();
    // 获取角点特征点云，用于边缘约束优化
    data.corner_cloud = livox_feature_extraction_->GetCornerFeature();
    // 将处理完成的雷达数据添加到缓冲队列中，等待后续处理
    lidar_buf_.push_back(data);

    // 多雷达系统坐标变换：如果不是主雷达（ID≠0），需要将点云变换到主雷达坐标系
    if (lidar_id != 0)
    {
      // 对原始点云进行坐标变换，从当前雷达坐标系转换到主雷达（lidar_0）坐标系
      pcl::transformPointCloud(*data.raw_cloud, *data.raw_cloud,
                              T_LktoL0_vec_[lidar_id]);
      // 对表面特征点云进行相同的坐标变换，保持几何一致性
      pcl::transformPointCloud(*data.surf_cloud, *data.surf_cloud,
                              T_LktoL0_vec_[lidar_id]);
      // 对角点特征点云进行坐标变换，统一到主雷达坐标系下
      pcl::transformPointCloud(*data.corner_cloud, *data.corner_cloud,
                              T_LktoL0_vec_[lidar_id]);
    }
  }

  void MsgManager::ImageMsgHandle(const sensor_msgs::ImageConstPtr &msg)
  {
    if (pub_img_.getNumSubscribers() != 0)
    {
      pub_img_.publish(msg);
    }

    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cvImgPtr->image.empty())
    {
      std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET << std::endl;
      return;
    }

    image_buf_.emplace_back();
    image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
    image_buf_.back().image = cvImgPtr->image;
    nerf_time_.push_back(image_buf_.back().timestamp);

    if (image_buf_.back().image.cols == 640 || image_buf_.back().image.cols == 1280)
    {
      cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
    }

    // // for tiers
    // if (image_buf_.back().image.cols == 1920)
    // {
    //   cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
    // }
  }

  void MsgManager::ImageMsgHandle(const sensor_msgs::CompressedImageConstPtr &msg)
  {
    if (pub_img_.getNumSubscribers() != 0)
    {
      cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      sensor_msgs::Image imgMsg = *(cvImgPtr->toImageMsg());
      imgMsg.header = msg->header; // 
      pub_img_.publish(msg);
    }

    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cvImgPtr->image.empty())
    {
      std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET << std::endl;
      return;
    }

    image_buf_.emplace_back();
    image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
    image_buf_.back().image = cvImgPtr->image;
    nerf_time_.push_back(image_buf_.back().timestamp);

    // std::cout << image_buf_.back().image.rows << " " << image_buf_.back().image.cols << std::endl;

    if (image_buf_.back().image.cols == 640 || image_buf_.back().image.cols == 1280)
    {
      cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
    }

    // // for mars
    // if (image_buf_.back().image.cols == 2448)
    // {
    //   // cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(1224, 1024), 0, 0, cv::INTER_LINEAR);
    //   cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(612, 512), 0, 0, cv::INTER_LINEAR);
    // }
  }

} // namespace cocolic
