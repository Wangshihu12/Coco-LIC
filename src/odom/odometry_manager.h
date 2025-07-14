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

#pragma once

#include <ros/ros.h>

#include <odom/msg_manager.h>
#include <odom/odometry_viewer.h>
#include <odom/trajectory_manager.h>

#include <imu/imu_state_estimator.h>
#include <imu/imu_initializer.h>
#include <lidar/lidar_handler.h>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <camera/r3live.hpp>

namespace cocolic
{

  enum KnotDensity
  {
    gear1 = 1, // 0.1
    gear2 = 2, // 0.05
    gear3 = 3, // 0.033
    gear4 = 4  // 0.025
  };
  class OdometryManager
  {
  public:
    OdometryManager(const YAML::Node &node, ros::NodeHandle &nh);

    void RunBag();

    void RunInSubscribeMode();

    double SaveOdometry();

    std::vector<int> cp_num_vec;

    // 根据IMU数据的运动强度决定B样条节点密度
    int GetKnotDensity(double gyro_norm, double acce_norm)
    {
      // 输入参数有效性检查
      if (gyro_norm < 0.0 || acce_norm < 0.0)
      {
        std::cout << RED << "gyro_norm/acce_norm is wrong!" << RESET << std::endl;
      }

      int gyro_density = -1, acce_density = -1;  // 陀螺仪和加速度计对应的密度档位

      // 去除重力影响：计算真正的加速度变化量
      // 减去重力模长，得到运动引起的加速度变化
      acce_norm = std::abs(acce_norm - gravity_norm_);
      // LOG(INFO) << "[acce_norm] " << acce_norm;
      
      // 根据加速度变化量分档：运动越剧烈，需要更密集的控制点
      if (acce_norm < 0.5)
      { // [0, 0.5) - 几乎静止或匀速运动
        acce_density = KnotDensity::gear1;  // 最低密度档位
      }
      else if (acce_norm < 1.0)
      { // [0.5, 1.0) - 轻微加速度变化
        acce_density = KnotDensity::gear2;  // 第二档密度
      }
      else if (acce_norm < 5.0)
      { // [1.0, 5.0) - 中等加速度变化
        acce_density = KnotDensity::gear3;  // 第三档密度
      }
      else
      { // [5.0, -) - 强烈加速度变化
        acce_density = KnotDensity::gear4;  // 最高密度档位
      }

      // LOG(INFO) << "[gyro_norm] " << gyro_norm;
      // 根据角速度大小分档：旋转越快，需要更密集的控制点
      if (gyro_norm < 0.5)
      { // [0, 0.5) - 几乎不旋转或缓慢旋转
        gyro_density = KnotDensity::gear1;  // 最低密度档位
      }
      else if (gyro_norm < 1.0)
      { // [0.5, 1.0) - 轻微旋转
        gyro_density = KnotDensity::gear2;  // 第二档密度
      }
      else if (gyro_norm < 5.0)
      { // [1.0, 5.0) - 中等旋转速度
        gyro_density = KnotDensity::gear3;  // 第三档密度
      }
      else
      { // [5.0, -) - 快速旋转
        gyro_density = KnotDensity::gear4;  // 最高密度档位
      }

      // 返回两个密度档位的最大值：取更高的密度要求
      // 这样可以确保轨迹能够准确描述最剧烈的运动分量
      return std::max(gyro_density, acce_density);
    };

  protected:
    bool CreateCacheFolder(const std::string &config_path,
                           const std::string &bag_path);

    void SolveLICO();

    void ProcessLICData();

    void ProcessImageData();

    bool PrepareTwoSegMsgs(int seg_idx);

    void UpdateTwoSeg();

    bool PrepareMsgs();

    void UpdateOneSeg();

    void SetInitialState();

    void PublishCloudAndTrajectory();

    void Publish3DGSMappingData(const NextMsgs& cur_msg);

  protected:
    OdometryMode odometry_mode_;

    MsgManager::Ptr msg_manager_;

    bool is_initialized_;
    IMUInitializer::Ptr imu_initializer_;

    Trajectory::Ptr trajectory_;
    TrajectoryManager::Ptr trajectory_manager_;

    LidarHandler::Ptr lidar_handler_;

    R3LIVE::Ptr camera_handler_;

    int64_t t_begin_add_cam_; // 

    OdometryViewer odom_viewer_;

    int update_every_k_knot_;

    /// [nurbs]
    double t_add_;
    int64_t t_add_ns_;
    bool non_uniform_;
    double distance0_;

    int cp_add_num_coarse_;
    int cp_add_num_refine_;

    int lidar_iter_;
    bool use_lidar_scale_;

    std::string cache_path_;

    double pasue_time_;

    TimeStatistics time_summary_;

    struct SysTimeOffset
    {
      SysTimeOffset(double t1, double t2, double t3, double t4)
          : timestamp(t1), t_lidar(t2), t_cam(t3), t_imu(t4) {}
      double timestamp = 0;
      double t_lidar = 0;
      double t_cam = 0;
      double t_imu = 0;
    };
    std::vector<SysTimeOffset> sys_t_offset_vec_;

  private:
    double gravity_norm_;

    int64_t traj_max_time_ns_cur;
    int64_t traj_max_time_ns_next;
    int64_t traj_max_time_ns_next_next;

    int cp_add_num_cur;
    int cp_add_num_next;
    int cp_add_num_next_next;

    bool is_evo_viral_;

    double ave_r_thresh_;
    double ave_a_thresh_;

    VPointCloud sub_map_cur_frame_point_;

    Eigen::aligned_vector<Eigen::Vector3d> v_points_;
    Eigen::aligned_vector<Eigen::Vector2d> px_obss_;

    Eigen::Matrix3d K_;

    bool if_3dgs_;
    int lidar_skip_;

    std::queue<int64_t> time_buf;  // img timestamp
    std::queue<LiDARFeature> lidar_buf;  // lidarfeature in local
    std::queue<cv::Mat> img_buf;  // undistorted
  };

} // namespace cocolic
