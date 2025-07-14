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

#include "trajectory.h"
#include <fstream>

namespace cocolic {

void Trajectory::GetIMUState(double time, IMUState &imu_state) const {
  // std::cout << "[time] " << time << std::endl;
  SE3d pose = GetIMUPoseNURBS(time);

  imu_state.timestamp = time;
  imu_state.q = pose.unit_quaternion();
  imu_state.p = pose.translation();

  int64_t time_ns = time * S_TO_NS;
  std::pair<int, double> su;  //i u
  bool flag = false;
  for (int i = 0; i < knts.size() - 1; i++) {
    if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
      su.first = i;
      su.second = 1.0 * (time_ns - knts[i]) / (knts[i + 1] - knts[i]);
      flag = true;
    }
  }
  if (!flag) std::cout << "[GetIMUState query wrong]\n";

  double delta_t = (knts[su.first + 1] - knts[su.first]) * NS_TO_S;
  Eigen::Matrix4d blend_mat = blending_mats[su.first - 3];

  su.first -= 3;
  imu_state.v = GetTransVelWorldNURBS(su, delta_t, blend_mat);
  // imu_state.bias;
  // imu_state.g;
}

void Trajectory::UndistortScan(const PosCloud &scan_raw,
                               const int64_t target_timestamp,
                               PosCloud &scan_in_target) const {
  scan_in_target.header = scan_raw.header;
  scan_in_target.resize(scan_raw.size());
  scan_in_target.is_dense = true;

  int start_idx = INT_MAX;
  {
    bool flag = false;
    int64_t time_ns = target_timestamp;
    for (int i = 0; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        start_idx = i;
        flag = true;
        break;
      }
    }
    if (!flag) std::cout << "[UndistortScan wrong]\n";
  }
  start_idx -= 2;
  if (start_idx < 0) start_idx = 0;

  SE3d pose_G_to_target = GetLidarPoseNURBS(target_timestamp, start_idx).inverse();  // TL0_G

  std::size_t cnt = 0;
  for (auto const &raw_p : scan_raw.points) {
    if (std::isnan(raw_p.x)) {
      scan_in_target.is_dense = false;
      // LOG(WARNING) << "[UndistortScan] input cloud exists NAN point";
      continue;
    }
    SE3d pose_Lk_to_G = GetLidarPoseNURBS(raw_p.timestamp, start_idx);  // TG_LK

    Eigen::Vector3d p_Lk(raw_p.x, raw_p.y, raw_p.z);
    Eigen::Vector3d point_out;
    point_out = pose_G_to_target * pose_Lk_to_G * p_Lk;

    PosPoint point;
    point.x = point_out(0);
    point.y = point_out(1);
    point.z = point_out(2);
    point.intensity = raw_p.intensity;
    point.timestamp = raw_p.timestamp;

    scan_in_target[cnt++] = point;
  }

  scan_in_target.resize(cnt);
}

// 在全局坐标系下对激光雷达扫描进行去畸变：消除运动引起的点云畸变
void Trajectory::UndistortScanInG(const PosCloud &scan_raw,           // 输入：原始畸变点云
                                  const int64_t scan_raw_timestamp,   // 输入：扫描时间戳
                                  PosCloud &scan_in_target) const {   // 输出：去畸变后的点云
  // 设置输出点云的基本属性
  scan_in_target.header = scan_raw.header;  // 复制头信息
  scan_in_target.resize(scan_raw.size());   // 预分配内存，大小与输入相同
  scan_in_target.is_dense = true;           // 初始假设输出点云是密集的（无NaN点）

  // 找到扫描时间戳在B样条节点序列中的起始索引
  int start_idx = INT_MAX;  // 初始化为最大值
  {
    bool flag = false;      // 标记是否找到对应的时间区间
    int64_t time_ns = scan_raw_timestamp;  // 扫描时间戳
    
    // 遍历B样条节点，找到包含当前时间戳的区间
    for (int i = 0; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {  // 如果时间戳在[knts[i], knts[i+1])区间内
        start_idx = i;    // 记录起始索引
        flag = true;      // 标记找到
        break;
      }
    }
    if (!flag) std::cout << "[UndistortScanInG wrong]\n";  // 如果没找到，输出错误信息
  }
  
  // 调整起始索引：向前偏移2个位置以确保B样条插值有足够的控制点
  start_idx -= 2;
  if (start_idx < 0) start_idx = 0;  // 确保索引不为负

  // 遍历原始点云中的每个点进行去畸变处理
  std::size_t cnt = 0;  // 有效点计数器
  for (auto const &raw_p : scan_raw.points) {
    // 检查点是否有效（不是NaN）
    if (std::isnan(raw_p.x)) {
      scan_in_target.is_dense = false;  // 标记输出点云不再是密集的
      // LOG(WARNING) << "[UndistortScanInG] input cloud exists NAN point";
      continue;  // 跳过无效点
    }
    
    // 获取该点采集时刻激光雷达到全局坐标系的位姿变换 TG_LK
    SE3d pose_Lk_to_G = GetLidarPoseNURBS(raw_p.timestamp, start_idx);

    // 将点从激光雷达坐标系变换到全局坐标系
    Eigen::Vector3d p_Lk(raw_p.x, raw_p.y, raw_p.z);  // 激光雷达坐标系下的点
    Eigen::Vector3d point_out;                         // 全局坐标系下的点
    point_out = pose_Lk_to_G * p_Lk;                   // 执行坐标变换

    // 构造输出点并保存其他属性
    PosPoint point;
    point.x = point_out(0);           // 全局坐标系下的x坐标
    point.y = point_out(1);           // 全局坐标系下的y坐标
    point.z = point_out(2);           // 全局坐标系下的z坐标
    point.intensity = raw_p.intensity; // 保持原始强度值
    point.timestamp = raw_p.timestamp; // 保持原始时间戳

    scan_in_target[cnt++] = point;    // 将处理后的点加入输出点云
  }

  // 调整输出点云大小为实际有效点的数量
  scan_in_target.resize(cnt);
}

SE3d Trajectory::GetSensorPose(const double timestamp,
                               const ExtrinsicParam &EP_StoI) const {
  double time_ns = timestamp * S_TO_NS + EP_StoI.t_offset_ns;

  if (!(time_ns >= this->minTimeNs() && time_ns < this->maxTimeNs())) {
    std::cout << time_ns << "; not in [" << this->minTimeNs() << ", "
              << this->maxTimeNs() << "]; "
              << "input time: " << timestamp
              << "[s]; t_offset: " << EP_StoI.t_offset_ns << " [ns]\n";
  }
  assert(time_ns >= this->minTimeNs() && time_ns < this->maxTimeNs() &&
         "[GetSensorPose] querry time not in range.");

  SE3d pose_I_to_G = this->poseNs(time_ns);
  SE3d pose_S_to_G = pose_I_to_G * EP_StoI.se3;
  return pose_S_to_G;
}

SE3d Trajectory::GetSensorPoseNURBS(const int64_t timestamp,
                               const ExtrinsicParam &EP_StoI) const {
  int64_t time_ns = timestamp;

  assert(time_ns >= 0 && time_ns < this->maxTimeNsNURBS() &&
         "[GetSensorPose] querry time not in range.");

  SE3d pose_I_to_G = this->poseNsNURBS(time_ns);

  SE3d pose_S_to_G = pose_I_to_G * EP_StoI.se3;
  return pose_S_to_G;
}

SE3d Trajectory::GetSensorPoseNURBS(const int64_t timestamp,
                               const ExtrinsicParam &EP_StoI, int start_idx) const {
  int64_t time_ns = timestamp;

  assert(time_ns >= 0 && time_ns < this->maxTimeNsNURBS() &&
         "[GetSensorPose] querry time not in range.");

  SE3d pose_I_to_G = this->poseNsNURBS(time_ns, start_idx);

  SE3d pose_S_to_G = pose_I_to_G * EP_StoI.se3;
  return pose_S_to_G;
}

void Trajectory::ToTUMTxt(std::string traj_path, int64_t maxtime, bool is_evo_viral, double dt) {
  std::ofstream outfile;
  outfile.open(traj_path);
  outfile.setf(std::ios::fixed);

  int64_t min_time = 0;
  int64_t max_time = maxtime;
  int64_t dt_ns = dt * S_TO_NS;
  SE3d start_end;
  SE3d start_pose = GetIMUPoseNsNURBS(min_time);
  for (int64_t t = min_time; t < max_time; t += dt_ns) {
    SE3d pose = GetIMUPoseNsNURBS(t);
    Eigen::Vector3d p = pose.translation();
    Eigen::Quaterniond q = pose.unit_quaternion();
    start_end = start_pose.inverse() * pose;

    /// for VIRAL
    if (is_evo_viral) {
      p = (q.toRotationMatrix() * Eigen::Vector3d(-0.293656, -0.012288, -0.273095) + p).eval();
    }
  
    double relative_bag_time = (data_start_time_ + t) * NS_TO_S;
    outfile.precision(9);
    outfile << relative_bag_time << " ";
    outfile.precision(5);
    outfile << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " "
            << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  outfile.close();
  std::cout << "\n🍺 Save trajectory at " << traj_path << std::endl;

  Eigen::AngleAxisd rotation_vector(start_end.unit_quaternion());
  std::cout << "   Start-to-end deviation: " << std::setprecision(3) << start_end.translation().norm() << "m, " << rotation_vector.angle() * 180 / M_PI  << "°." << std::endl;
}

}  // namespace cocolic
