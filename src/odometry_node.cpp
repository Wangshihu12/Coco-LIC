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

#include <ros/package.h>
#include <ros/ros.h>

#include <odom/odometry_manager.h>

using namespace cocolic;

/**
 * [功能描述]：Coco-LIC系统主入口函数，初始化ROS节点并启动连续时间紧耦合LiDAR-惯性-相机里程计
 * @param argc：命令行参数数量
 * @param argv：命令行参数数组，包含程序名称和其他参数
 * @return 程序执行状态码，0表示正常结束
 */
int main(int argc, char **argv) {
  // 初始化Google日志系统，用于程序运行过程中的日志记录
  google::InitGoogleLogging(argv[0]);

  // 初始化ROS节点，节点名称为"cocolic"，用于与ROS通信框架交互
  ros::init(argc, argv, "cocolic");
  // 创建私有节点句柄，用于获取节点参数和发布/订阅话题
  ros::NodeHandle nh("~");

  // 声明配置文件路径变量
  std::string config_path;
  // 从ROS参数服务器获取配置文件路径，如果未设置则使用默认值"ct_odometry.yaml"
  nh.param<std::string>("config_path", config_path, "ct_odometry.yaml");
  // 输出加载的配置文件路径信息到ROS日志系统
  ROS_INFO("Odometry load %s.", config_path.c_str());

  // 使用YAML库加载配置文件，解析系统参数配置
  YAML::Node config_node = YAML::LoadFile(config_path);

  // 注释掉的代码段：用于设置Google日志输出路径和颜色显示
  // std::string log_path = config_node["log_path"].as<std::string>();
  // FLAGS_log_dir = log_path;
  // FLAGS_colorlogtostderr = true;
  
  // 输出系统启动信息，使用表情符号增强用户体验
  std::cout << "\n🥥 Start Coco-LIC Odometry 🥥";

  // 创建里程计管理器对象，传入配置节点和ROS节点句柄进行初始化
  OdometryManager odom_manager(config_node, nh);
  // 启动数据包处理流程，开始执行SLAM算法
  odom_manager.RunBag();

  // 保存里程计结果并获取轨迹的最大时间戳
  double t_traj_max = odom_manager.SaveOdometry();
  // 输出程序完成信息
  std::cout << "\n✨ All Done.\n\n";

  // 返回程序正常结束状态码
  return 0;
}
