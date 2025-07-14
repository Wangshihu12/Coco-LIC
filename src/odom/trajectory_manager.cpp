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

#include <odom/factor/analytic_diff/image_feature_factor.h>
#include <odom/factor/analytic_diff/trajectory_value_factor.h>
#include <odom/trajectory_manager.h>
#include <ros/assert.h>
#include <utils/log_utils.h>

#include <fstream>
std::fstream myfile_t_ba;
namespace cocolic
{

  // TrajectoryManager 构造函数：初始化轨迹管理器
  TrajectoryManager::TrajectoryManager(const YAML::Node &node,
    const std::string &config_path,
    Trajectory::Ptr trajectory)
  : verbose(false),                    // 详细输出模式，默认关闭
  cur_img_time_(-1),                 // 当前图像时间戳，初始化为-1
  process_cur_img_(false),           // 是否处理当前图像，默认为false
  opt_weight_(OptWeight(node)),      // 优化权重参数，从配置文件中读取
  trajectory_(trajectory),           // 轨迹对象指针
  lidar_marg_info(nullptr),         // 激光雷达边缘化信息，初始化为空
  cam_marg_info(nullptr)            // 相机边缘化信息，初始化为空
  {
  // 加载IMU配置文件
  std::string imu_yaml = node["imu_yaml"].as<std::string>();
  YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);

  // 创建IMU状态估计器
  imu_state_estimator_ = std::make_shared<ImuStateEstimator>(imu_node);

  // 从配置文件中读取是否使用初始陀螺仪偏置
  if_use_init_bg_ = imu_node["if_use_init_bg"].as<bool>();

  // 初始化激光雷达先验控制点ID对，默认为(0, 0)
  lidar_prior_ctrl_id = std::make_pair(0, 0);

  // 初始化因子信息，包括相机和激光雷达的外参、图像权重、局部速度信息
  InitFactorInfo(trajectory_->GetSensorEP(CameraSensor),      // 相机外参
  trajectory_->GetSensorEP(LiDARSensor),       // 激光雷达外参
  opt_weight_.image_weight,                    // 图像权重
  opt_weight_.local_velocity_info_vec);        // 局部速度信息向量

  // 初始化分割参数
  division_ = 0;

  // 启用边缘化功能
  use_marg_ = true;

  // 初始化优化统计信息
  opt_cnt = 0;          // 优化计数器
  t_opt_sum = 0.0;      // 优化总时间

  // 清空视觉观测容器
  v_points_.clear();    // 清空3D点容器
  px_obss_.clear();     // 清空像素观测容器
  }

  // 初始化因子信息：设置优化中各种因子的参数和权重
  void TrajectoryManager::InitFactorInfo(
    const ExtrinsicParam &Ep_CtoI,        // 相机到IMU的外参
    const ExtrinsicParam &Ep_LtoI,        // 激光雷达到IMU的外参
    const double image_feature_weight,    // 图像特征权重
    const Eigen::Vector3d &local_velocity_weight)  // 局部速度权重
  {
  // 如果图像特征权重大于阈值，则初始化相机相关的因子
  if (image_feature_weight > 1e-5)
  {
    // 创建权重信息矩阵的平方根（用于优化中的权重计算）
    Eigen::Matrix2d sqrt_info =
        image_feature_weight * Eigen::Matrix2d::Identity();

    // 设置图像特征因子的外参和权重信息
    analytic_derivative::ImageFeatureFactor::SetParam(Ep_CtoI.so3, Ep_CtoI.p);
    analytic_derivative::ImageFeatureFactor::sqrt_info = sqrt_info;

    // 设置图像3D到2D投影因子的外参和权重信息
    analytic_derivative::Image3D2DFactor::SetParam(Ep_CtoI.so3, Ep_CtoI.p);
    analytic_derivative::Image3D2DFactor::sqrt_info = sqrt_info;

    // 设置单帧图像特征因子的外参和权重信息
    analytic_derivative::ImageFeatureOnePoseFactor::SetParam(Ep_CtoI.so3,
                                                            Ep_CtoI.p);
    analytic_derivative::ImageFeatureOnePoseFactor::sqrt_info = sqrt_info;

    // 设置图像深度因子的权重信息
    analytic_derivative::ImageDepthFactor::sqrt_info = sqrt_info;

    // 设置对极线因子的外参（用于立体视觉约束）
    analytic_derivative::EpipolarFactor::SetParam(Ep_CtoI.so3, Ep_CtoI.p);
  }

  // 设置LOAM特征优化地图位姿因子的外参（激光雷达点云配准）
  analytic_derivative::LoamFeatureOptMapPoseFactor::SetParam(Ep_LtoI.so3,
                                                            Ep_LtoI.p);

  // 设置相对LOAM特征因子的外参（激光雷达帧间约束）
  analytic_derivative::RalativeLoamFeatureFactor::SetParam(Ep_LtoI.so3,
                                                          Ep_LtoI.p);
  }

  // 设置系统初始状态：配置轨迹的初始参数和IMU偏置
  void TrajectoryManager::SetSystemState(const SystemState &sys_state, double distance0)
  {
    // 设置重力向量（全局坐标系下的重力加速度）
    gravity_ = sys_state.g;

    // 设置原始位姿（初始位置和姿态）
    SetOriginalPose(sys_state.q, sys_state.p);

    // 添加B样条的节点（knots）来构建轨迹
    trajectory_->AddKntNs(0.0 * S_TO_NS);       // 添加节点t3（t0、t1、t2已经在构造函数中添加）
    trajectory_->AddKntNs(distance0 * S_TO_NS); // 添加节点t4，distance0为初始距离参数
    
    // 设置轨迹的最大时间为最后一个节点的时间
    trajectory_->SetMaxTimeNsNURBS(trajectory_->knts.back());

    // 创建初始旋转矩阵
    SO3d R0(sys_state.q);
    
    // 将所有B样条节点的旋转设置为初始旋转
    for (size_t i = 0; i < trajectory_->numKnots(); i++)
    {
      trajectory_->setKnotSO3(R0, i);  // 设置第i个节点的旋转为R0
    }
    // LOG(INFO) << "[debug numKnots] " << trajectory_->numKnots(); // 应该是4个节点

    // 设置IMU偏置相关的时间参数
    tparam_.last_bias_time = trajectory_->maxTimeNsNURBS();  // 上一个偏置时间
    tparam_.cur_bias_time = trajectory_->maxTimeNsNURBS();   // 当前偏置时间

    // 设置IMU偏置值
    all_imu_bias_[tparam_.last_bias_time] = sys_state.bias;  // 存储初始偏置
    
    // 如果不使用初始陀螺仪偏置，则将偏置设为零
    if (!if_use_init_bg_)
    {
      all_imu_bias_[tparam_.last_bias_time].gyro_bias = Eigen::Vector3d::Zero();   // 陀螺仪偏置设为零
      all_imu_bias_[tparam_.last_bias_time].accel_bias = Eigen::Vector3d::Zero();  // 加速度计偏置设为零
    }
  }

  void TrajectoryManager::SetOriginalPose(Eigen::Quaterniond q,
                                          Eigen::Vector3d p)
  {
    original_pose_.orientation.setQuaternion(q);
    original_pose_.position = p;
  }

  void TrajectoryManager::AddIMUData(const IMUData &data)
  {
    if (trajectory_->GetDataStartTime() < 0)
    {
      trajectory_->SetDataStartTime(data.timestamp);
    }
    imu_data_.emplace_back(data);
    imu_data_.back().timestamp -= trajectory_->GetDataStartTime();

    imu_state_estimator_->FeedIMUData(imu_data_.back());
  }

  void TrajectoryManager::AddPoseData(const PoseData &data)
  {
    pose_data_.emplace_back(data);
    pose_data_.back().timestamp -= trajectory_->GetDataStartTime();
  }

  void TrajectoryManager::RemoveIMUData(int64_t t_window_min)
  {
    if (t_window_min < 0)
      return;

    // https://stackoverflow.com/questions/991335/
    // how-to-erase-delete-pointers-to-objects-stored-in-a-vector
    for (auto iter = imu_data_.begin(); iter != imu_data_.end();)
    {
      if (iter->timestamp < t_window_min)
      {
        iter = imu_data_.erase(iter);
      }
      else
      {
        break;
      }
    }
  }

  void TrajectoryManager::RemovePoseData(int64_t t_window_min)
  {
    if (t_window_min < 0)
      return;

    // https://stackoverflow.com/questions/991335/
    // how-to-erase-delete-pointers-to-objects-stored-in-a-vector
    for (auto iter = pose_data_.begin(); iter != pose_data_.end();)
    {
      if (iter->timestamp < t_window_min)
      {
        iter = pose_data_.erase(iter);
      }
      else
      {
        break;
      }
    }
  }

  // 更新LIO中使用的IMU数据范围：确定优化时间区间内的IMU数据索引
  void TrajectoryManager::UpdateIMUInlio()
  {
    // 获取当前优化的时间区间
    int64_t t_min = opt_min_t_ns;  // 优化起始时间
    int64_t t_max = opt_max_t_ns;  // 优化结束时间

    // 正向遍历：找到第一个时间戳 >= t_min 的IMU数据
    for (auto iter = imu_data_.begin(); iter != imu_data_.end(); ++iter)
    {
      if (iter->timestamp >= t_min)  // 如果IMU时间戳大于等于优化起始时间
      {
        if (iter->timestamp >= t_max)  // 如果IMU时间戳已经超过优化结束时间
        {
          continue;  // 跳过这个数据点
        }
        // 记录第一个有效IMU数据的索引和时间戳
        tparam_.lio_imu_idx[0] = std::distance(imu_data_.begin(), iter);  // 起始索引
        tparam_.lio_imu_time[0] = iter->timestamp;                       // 起始时间
        break;  // 找到第一个有效点后退出循环
      }
    }

    // 反向遍历：找到最后一个时间戳 < t_max 的IMU数据
    for (auto rter = imu_data_.rbegin(); rter != imu_data_.rend(); ++rter)
    {
      if (rter->timestamp < t_max)  // 如果IMU时间戳小于优化结束时间
      {
        // 记录最后一个有效IMU数据的索引
        // rter.base() - 1 是因为反向迭代器到正向迭代器的转换
        tparam_.lio_imu_idx[1] = std::distance(imu_data_.begin(), rter.base()) - 1;  // 结束索引
        tparam_.lio_imu_time[1] = rter->timestamp;                                   // 结束时间
        break;  // 找到最后一个有效点后退出循环
      }
    }
  }

  // 预测轨迹：基于IMU数据和先验信息进行轨迹扩展和初步优化
  void TrajectoryManager::PredictTrajectory(int64_t scan_time_min, int64_t scan_time_max,
    int64_t traj_max_time_ns, int knot_add_num, bool non_uniform)
  {
  // 检查IMU数据的有效性
  if (imu_data_.empty() || imu_data_.size() == 1)
  {
  // 如果IMU数据为空或只有一个数据点，无法进行轨迹预测
  // LOG(ERROR) << "[AppendWithIMUData] IMU data empty! ";
  return;
  }

  /// 新添加的时间区间：[opt_min_t_ns, opt_max_t_ns)
  opt_min_t_ns = trajectory_->maxTimeNsNURBS();  // 优化起始时间 = 当前轨迹的最大时间

  /// 通过添加控制点来扩展轨迹
  trajectory_->SetMaxTimeNsNURBS(traj_max_time_ns);  // 设置轨迹的新最大时间
  opt_max_t_ns = trajectory_->maxTimeNsNURBS();      // 优化结束时间 = 更新后的最大时间
  SE3d last_knot = trajectory_->getLastKnot();       // 获取最后一个控制点的位姿
  trajectory_->extendKnotsTo(knot_add_num, last_knot); // 扩展轨迹，添加指定数量的控制点

  ////// 为控制点着色用于可视化
  int intensity = 0;  // 强度值，用于区分不同密度的控制点
  if (knot_add_num == 1)
  {
  intensity = 100;    // 1个控制点：低密度（浅色）
  }
  else if (knot_add_num == 2)
  {
  intensity = 200;    // 2个控制点：中低密度
  }
  else if (knot_add_num == 3)
  {
  intensity = 300;    // 3个控制点：中高密度
  }
  else if (knot_add_num == 4)
  {
  intensity = 400;    // 4个控制点：高密度（深色）
  }
  // 为新添加的控制点分配颜色强度
  for (int i = 0; i < knot_add_num; i++)
  {
  trajectory_->intensity_map[trajectory_->numKnots() + i] = intensity;
  }
  ////// 控制点着色结束

  // LOG(INFO) << "[max_time_ns] " << opt_max_t_ns;
  // LOG(INFO) << "[numKnots aft extension] " << trajectory_->numKnots();

  // 更新IMU偏置相关的时间参数
  tparam_.last_bias_time = tparam_.cur_bias_time;  // 上一个偏置时间 = 当前偏置时间
  tparam_.cur_bias_time = opt_max_t_ns;            // 当前偏置时间 = 优化结束时间
  // LOG(INFO) << "[last_bias_time] " << tparam_.last_bias_time << " "
  //           << "[cur_bias_time] " << tparam_.cur_bias_time;

  // 更新当前扫描的时间范围
  tparam_.UpdateCurScan(scan_time_min, scan_time_max);

  // 确定参与此次优化的IMU数据
  UpdateIMUInlio();

  /// 执行优化：使用IMU预积分初始化轨迹
  InitTrajWithPropagation();
  }

  // 使用IMU传播初始化轨迹：基于IMU预积分对新扩展的轨迹段进行初始化
  void TrajectoryManager::InitTrajWithPropagation()
  {
    // 设置轨迹估计器的优化选项
    TrajectoryEstimatorOptions option;
    option.lock_ab = true;          // 锁定加速度计偏置（不优化）
    option.lock_wb = true;          // 锁定陀螺仪偏置（不优化）
    option.lock_g = true;           // 锁定重力向量（不优化）
    option.lock_tran = false;       // 不锁定平移（允许优化位置）注意：这很重要
    option.show_residual_summary = verbose;  // 根据verbose标志决定是否显示残差摘要
    
    // 创建轨迹估计器对象
    TrajectoryEstimator::Ptr estimator(
        new TrajectoryEstimator(trajectory_, option, "Init Traj"));

    // 设置固定的控制点索引：第3个控制点作为参考点固定不动
    estimator->SetFixedIndex(3);

    // [0] 添加先验因子：如果存在激光雷达边缘化信息
    if (true && lidar_marg_info)
    {
      // 添加边缘化因子，包含之前优化的先验信息
      estimator->AddMarginalizationFactor(lidar_marg_info,          // 边缘化信息
                                          lidar_marg_parameter_blocks);  // 相关的参数块
    }

    // [1] 添加IMU因子：使用指定时间范围内的IMU数据
    // 获取最新的IMU偏置参数指针
    double *para_bg = all_imu_bias_.rbegin()->second.gyro_bias.data();   // 陀螺仪偏置参数
    double *para_ba = all_imu_bias_.rbegin()->second.accel_bias.data();  // 加速度计偏置参数
    
    // 遍历指定范围内的IMU数据
    for (int i = tparam_.lio_imu_idx[0]; i <= tparam_.lio_imu_idx[1]; ++i)
    {
      // 跳过时间范围外的IMU数据
      if (imu_data_.at(i).timestamp < opt_min_t_ns)   // 小于优化起始时间
        continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)  // 大于等于优化结束时间
        continue;
      
      // 添加IMU测量的解析NURBS因子
      estimator->AddIMUMeasurementAnalyticNURBS(imu_data_.at(i),        // IMU数据
                                                para_bg,                 // 陀螺仪偏置
                                                para_ba,                 // 加速度计偏置
                                                gravity_.data(),         // 重力向量 (0, 0, 9.8)
                                                opt_weight_.imu_info_vec); // IMU权重信息
    }

    // 求解优化问题
    ceres::Solver::Summary summary = estimator->Solve(50, false);  // 最大50次迭代，不使用线程
    
    // 统计初始化次数（用于调试）
    static int init_cnt = 0;
    init_cnt++;
    // LOG(INFO) << init_cnt << " TrajInitSolver " << summary.BriefReport();
    // LOG(INFO) << init_cnt << " TrajInit Successful/Unsuccessful steps: "
    //           << summary.num_successful_steps << "/"
    //           << summary.num_unsuccessful_steps;
  }

  // 使用LIC数据更新轨迹：融合激光雷达、IMU、相机数据进行轨迹优化
  bool TrajectoryManager::UpdateTrajectoryWithLIC(
      int lidar_iter,                                                    // 激光雷达迭代次数
      int64_t img_time_stamp,                                           // 图像时间戳
      const Eigen::aligned_vector<PointCorrespondence> &point_corrs,   // 激光雷达点对应关系
      const Eigen::aligned_vector<Eigen::Vector3d> &pnp_3ds,          // PnP算法的3D点
      const Eigen::aligned_vector<Eigen::Vector2d> &pnp_2ds,          // PnP算法的2D点
      const int iteration)                                              // 优化迭代次数
  {
    // 检查输入数据的有效性
    if (point_corrs.empty() || imu_data_.empty() || imu_data_.size() == 1)
    {
      // 如果激光雷达对应关系为空或IMU数据不足，无法进行优化
      // LOG(WARNING) << " input empty data " << point_corrs.size() << ", "
      //              << imu_data_.size();
      return false;
    }

    // LOG(INFO) << "[point_corrs size] " << point_corrs.size();
    // LOG(INFO) << "[opt_domain]: "
    //           << "[" << opt_min_t_ns * NS_TO_S << ", " << opt_max_t_ns * NS_TO_S << ")";

    // 设置IMU偏置参数：管理两个时间段的偏置估计
    IMUBias last_bias = all_imu_bias_.rbegin()->second;  // 获取最新的IMU偏置
    all_imu_bias_[tparam_.cur_bias_time] = last_bias;    // 为当前时间设置偏置

    // 创建偏置参数指针映射
    std::map<int, double *> para_bg_vec;  // 陀螺仪偏置参数
    std::map<int, double *> para_ba_vec;  // 加速度计偏置参数
    {
      auto &bias0 = all_imu_bias_[tparam_.last_bias_time];  // 上一个时间段的偏置 bi
      para_bg_vec[0] = bias0.gyro_bias.data();
      para_ba_vec[0] = bias0.accel_bias.data();

      auto &bias1 = all_imu_bias_[tparam_.cur_bias_time];   // 当前时间段的偏置 bj
      para_bg_vec[1] = bias1.gyro_bias.data();
      para_ba_vec[1] = bias1.accel_bias.data();
    }

    // 配置轨迹估计器选项
    TrajectoryEstimatorOptions option;
    option.lock_ab = false;              // 不锁定加速度计偏置（允许优化）
    option.lock_wb = false;              // 不锁定陀螺仪偏置（允许优化）
    option.lock_g = true;                // 锁定重力向量（不优化）
    option.show_residual_summary = verbose;  // 根据verbose标志显示残差摘要

    // 创建轨迹估计器
    TrajectoryEstimator::Ptr estimator(
        new TrajectoryEstimator(trajectory_, option, "Before LIO"));

    // 设置固定的控制点索引：第3个控制点作为参考点
    estimator->SetFixedIndex(3);

    // [0] 添加先验因子：使用边缘化信息作为先验约束
    if (true && lidar_marg_info)
    {
      estimator->AddMarginalizationFactor(lidar_marg_info,           // 边缘化信息
                                          lidar_marg_parameter_blocks);  // 相关参数块
    }

    // [1] 添加激光雷达因子：建立点云配准约束
    // 获取激光雷达到IMU的外参
    SO3d S_LtoI = trajectory_->GetSensorEP(LiDARSensor).so3;        // 旋转外参
    Eigen::Vector3d p_LinI = trajectory_->GetSensorEP(LiDARSensor).p;  // 平移外参
    
    // 全局坐标系到地图坐标系的变换（这里设为单位变换）
    SO3d S_GtoM = SO3d(Eigen::Quaterniond::Identity());
    Eigen::Vector3d p_GinM = Eigen::Vector3d::Zero();

    // 遍历所有激光雷达点对应关系
    for (const auto &v : point_corrs)
    {
      // 筛选优化时间范围内的点
      if (v.t_point < opt_min_t_ns)      continue;  // 小于优化起始时间
      if (v.t_point >= opt_max_t_ns)     continue;  // 大于等于优化结束时间
      if (v.t_point < tparam_.last_scan[1]) continue;  // 小于上一扫描结束时间

      if (use_lidar_scale)  // 如果使用激光雷达尺度权重
      {
        // 添加LOAM测量因子（带自适应权重）
        estimator->AddLoamMeasurementAnalyticNURBS(v, S_GtoM, p_GinM, S_LtoI, p_LinI,
                                                  opt_weight_.lidar_weight * v.scale);
      }
      else  // 使用固定权重
      {
        // 添加LOAM测量因子（固定权重）
        estimator->AddLoamMeasurementAnalyticNURBS(v, S_GtoM, p_GinM, S_LtoI, p_LinI,
                                                  opt_weight_.lidar_weight);
      }
    }

    // [2] 添加IMU因子：提供运动约束
    for (int i = tparam_.lio_imu_idx[0]; i < tparam_.lio_imu_idx[1]; ++i)
    {
      // 筛选优化时间范围内的IMU数据
      if (imu_data_.at(i).timestamp < opt_min_t_ns)   continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)  continue;
      
      // 添加IMU测量因子
      estimator->AddIMUMeasurementAnalyticNURBS(imu_data_.at(i),      // IMU数据
                                                para_bg_vec[0],       // 陀螺仪偏置
                                                para_ba_vec[0],       // 加速度计偏置  
                                                gravity_.data(),      // 重力向量
                                                opt_weight_.imu_info_vec);  // IMU权重
    }

    /// [3] 添加偏置因子：约束IMU偏置的变化
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();      // 偏置协方差
    Eigen::Matrix<double, 6, 6> noise_covariance = Eigen::Matrix<double, 6, 6>::Zero(); // 噪声协方差
    
    // 设置偏置噪声协方差
    noise_covariance.block<3, 3>(0, 0) = (opt_weight_.imu_noise.sigma_wb_discrete * opt_weight_.imu_noise.sigma_wb_discrete) * Eigen::Matrix3d::Identity();  // 陀螺仪偏置噪声
    noise_covariance.block<3, 3>(3, 3) = (opt_weight_.imu_noise.sigma_ab_discrete * opt_weight_.imu_noise.sigma_ab_discrete) * Eigen::Matrix3d::Identity();  // 加速度计偏置噪声
    
    // 计算偏置的累积协方差（随时间传播）
    for (int i = tparam_.lio_imu_idx[0] + 1; i < tparam_.lio_imu_idx[1]; ++i)
    {
      if (imu_data_.at(i - 1).timestamp < opt_min_t_ns)  continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)     continue;
      
      double dt = (imu_data_[i].timestamp - imu_data_[i - 1].timestamp) * NS_TO_S;  // 时间间隔
      
      // 状态转移矩阵F（偏置模型为随机游走，F为单位矩阵）
      Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
      F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
      
      // 噪声传播矩阵G
      Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Zero();
      G.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt;
      G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt;
      
      // 协方差传播：P = F*P*F^T + G*Q*G^T
      covariance = F * covariance * F.transpose() + G * noise_covariance * G.transpose();
    }

    // 计算信息矩阵的平方根（用于优化）
    Eigen::Matrix<double, 6, 6> sqrt_info_mat = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(covariance.inverse()).matrixL().transpose();
    sqrt_info_ << sqrt_info_mat(0, 0), sqrt_info_mat(1, 1), sqrt_info_mat(2, 2), sqrt_info_mat(3, 3), sqrt_info_mat(4, 4), sqrt_info_mat(5, 5);
    
    // 添加偏置约束因子
    estimator->AddBiasFactor(para_bg_vec[0], para_bg_vec[1],  // 陀螺仪偏置（前后时刻）
                            para_ba_vec[0], para_ba_vec[1],  // 加速度计偏置（前后时刻）
                            1, sqrt_info_);                  // 权重和信息矩阵

    /// [4] 添加PnP因子：视觉重投影约束
    v_points_.clear();
    px_obss_.clear();
    if (pnp_3ds.size() != 0)  // 如果有视觉观测数据
    {
      v_points_ = pnp_3ds;     // 保存3D点
      px_obss_ = pnp_2ds;      // 保存2D观测
      process_cur_img_ = true;  // 标记处理当前图像
      cur_img_time_ = img_time_stamp;  // 保存图像时间戳
      
      // 添加PnP约束因子
      for (int i = 0; i < pnp_3ds.size(); i++)
      {
        estimator->AddPnPMeasurementAnalyticNURBS(
            pnp_3ds[i], pnp_2ds[i],                              // 3D-2D对应点
            img_time_stamp,                                      // 图像时间戳
            trajectory_->GetSensorEP(CameraSensor).so3,          // 相机外参旋转
            trajectory_->GetSensorEP(CameraSensor).p,            // 相机外参平移
            K_, opt_weight_.image_weight);                       // 相机内参和权重
      }
    }
    else
    {
      process_cur_img_ = false;  // 不处理当前图像
    }

    // 执行优化求解
    TicToc t_opt;  // 计时器
    static int loam_cnt = 0;
    ceres::Solver::Summary summary = estimator->Solve(iteration, false);  // 求解优化问题
    double opt_time = t_opt.toc();  // 获取优化耗时
    
    // 输出调试信息
    // LOG(INFO) << "[t_opt] " << opt_time << std::endl;
    // LOG(INFO) << "LoamSolver " << summary.BriefReport();
    // LOG(INFO) << ++loam_cnt << " UpdateLio Successful/Unsuccessful steps: "
    //           << summary.num_successful_steps << "/"
    //           << summary.num_unsuccessful_steps;

    // 更新统计信息
    opt_cnt++;              // 优化次数计数
    t_opt_sum += opt_time;  // 累计优化时间

    // 输出更新后的IMU偏置（调试用）
    // LOG(INFO) << "[gyro_bias_new] " << all_imu_bias_.rbegin()->second.gyro_bias.x() << " "
    //           << all_imu_bias_.rbegin()->second.gyro_bias.y() << " "
    //           << all_imu_bias_.rbegin()->second.gyro_bias.z();
    // LOG(INFO) << "[acce_bias_new] " << all_imu_bias_.rbegin()->second.accel_bias.x() << " "
    //           << all_imu_bias_.rbegin()->second.accel_bias.y() << " "
    //           << all_imu_bias_.rbegin()->second.accel_bias.z();

    return true;  // 优化成功
  }

  void TrajectoryManager::UpdateLiDARAttribute(double scan_time_min,
                                               double scan_time_max)
  {
    if (trajectory_->maxTimeNsNURBS() > 25 * S_TO_NS)
    {
      int64_t t = trajectory_->maxTimeNsNURBS() - 15 * S_TO_NS;
      RemoveIMUData(t);
      RemovePoseData(t);
    }
  }

  void TrajectoryManager::UpdateLICPrior(
      const Eigen::aligned_vector<PointCorrespondence> &point_corrs)
  {
    TrajectoryEstimatorOptions option;
    option.is_marg_state = true;

    TrajectoryEstimator::Ptr estimator(
        new TrajectoryEstimator(trajectory_, option));  // AddControlPoint

    // construct a new prior
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();

    // prepare the control points and biases to be marginalized
    int lhs_idx = trajectory_->numKnots() - 1 - division_ - 2;  // retain the last 3 control points in this optimization; remember, cubic spline is adopted
    int rhs_idx = trajectory_->numKnots() - 4;

    auto &last_bias = all_imu_bias_[tparam_.last_bias_time];  // marginalize the bias bi
    auto &cur_bias = all_imu_bias_[tparam_.cur_bias_time];
    std::vector<double *> drop_param;
    for (int i = lhs_idx; i <= rhs_idx; i++)
    {
      drop_param.emplace_back(trajectory_->getKnotSO3(i).data());
      drop_param.emplace_back(trajectory_->getKnotPos(i).data());
    }
    drop_param.emplace_back(last_bias.gyro_bias.data());
    drop_param.emplace_back(last_bias.accel_bias.data());

    // [0] prior factor marginalization
    if (lidar_marg_info)
    {
      std::vector<int> drop_set;
      for (int i = 0; i < lidar_marg_parameter_blocks.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (lidar_marg_parameter_blocks[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        MarginalizationFactor *cost_function = new MarginalizationFactor(lidar_marg_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_Prior, cost_function, NULL,
                                                                       lidar_marg_parameter_blocks, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    // [1] imu factor marginalization
    for (int i = tparam_.lio_imu_idx[0]; i < tparam_.lio_imu_idx[1]; ++i)
    {
      if (imu_data_.at(i).timestamp < opt_min_t_ns)
        continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)
        continue;
      int64_t time_ns = imu_data_.at(i).timestamp;
      std::pair<int, double> su; // i u
      trajectory_->GetIdxT(time_ns, su);
      Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
      Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];
      std::vector<double *> vec;
      estimator->AddControlPointsNURBS(su.first - 3, vec);
      estimator->AddControlPointsNURBS(su.first - 3, vec, true);
      vec.emplace_back(last_bias.gyro_bias.data());
      vec.emplace_back(last_bias.accel_bias.data());

      std::vector<int> drop_set;
      for (int i = 0; i < vec.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (vec[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        ceres::CostFunction *cost_function = new analytic_derivative::IMUFactorNURBS(
            time_ns, imu_data_.at(i), gravity_, opt_weight_.imu_info_vec, trajectory_->knts, su,
            blending_matrix, cumulative_blending_matrix);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_IMU, cost_function, NULL,
                                                                       vec, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    // [2] lidar factor marginalization
    SO3d S_LtoI = trajectory_->GetSensorEP(LiDARSensor).so3;
    Eigen::Vector3d p_LinI = trajectory_->GetSensorEP(LiDARSensor).p;
    SO3d S_GtoM = SO3d(Eigen::Quaterniond::Identity());
    Eigen::Vector3d p_GinM = Eigen::Vector3d::Zero();
    for (const auto &v : point_corrs)
    {
      if (v.t_point < opt_min_t_ns)
        continue;
      if (v.t_point >= opt_max_t_ns)
        continue;
      if (v.t_point < tparam_.last_scan[1])
        continue;
      int64_t time_ns = v.t_point;
      std::pair<int, double> su; // i and u
      trajectory_->GetIdxT(time_ns, su);
      Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
      Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];
      std::vector<double *> vec;
      estimator->AddControlPointsNURBS(su.first - 3, vec);
      estimator->AddControlPointsNURBS(su.first - 3, vec, true);

      std::vector<int> drop_set;
      for (int i = 0; i < vec.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (vec[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        double weight = opt_weight_.lidar_weight;
        if (use_lidar_scale)
        {
          weight *= v.scale;
        }
        ceres::CostFunction *cost_function = new analytic_derivative::LoamFeatureFactorNURBS(
            time_ns, v, su, blending_matrix, cumulative_blending_matrix,
            S_GtoM, p_GinM, S_LtoI, p_LinI, weight);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_LiDAR, cost_function, NULL,
                                                                       vec, drop_set);
        int num_residuals = cost_function->num_residuals();
        Eigen::MatrixXd residuals;
        residuals.setZero(num_residuals, 1);
        cost_function->Evaluate(vec.data(), residuals.data(), nullptr);
        double dist = (residuals / weight).norm();
        if (dist < 0.05)
        // if (dist < 0.01)
        {
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
      }
    }

    // [3] bias factor marginalization
    std::vector<double *> vec;
    vec.emplace_back(last_bias.gyro_bias.data());
    vec.emplace_back(cur_bias.gyro_bias.data());
    vec.emplace_back(last_bias.accel_bias.data());
    vec.emplace_back(cur_bias.accel_bias.data());

    std::vector<int> drop_set;
    drop_set.emplace_back(0); // bgi
    drop_set.emplace_back(2); // bai

    analytic_derivative::BiasFactor *cost_function = new analytic_derivative::BiasFactor(1, sqrt_info_);
    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_Bias, cost_function, NULL,
                                                                   vec, drop_set);
    marginalization_info->addResidualBlockInfo(residual_block_info);

    /// [4] pnp factor marginalization
    if (process_cur_img_ && v_points_.size() != 0)
    {
      int64_t time_ns = cur_img_time_;
      std::pair<int, double> su; // i和u
      trajectory_->GetIdxT(time_ns, su);
      Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
      Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];
      std::vector<double *> vec;
      estimator->AddControlPointsNURBS(su.first - 3, vec);
      estimator->AddControlPointsNURBS(su.first - 3, vec, true);

      std::vector<int> drop_set;
      for (int i = 0; i < vec.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (vec[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        Eigen::Matrix3d K;
        for (int i = 0; i < v_points_.size(); i++)
        {
          ceres::CostFunction *cost_function = new analytic_derivative::PnPFactorNURBS(
              time_ns, su,
              blending_matrix, cumulative_blending_matrix,
              v_points_[i], px_obss_[i],
              trajectory_->GetSensorEP(CameraSensor).so3,
              trajectory_->GetSensorEP(CameraSensor).p,
              K_, opt_weight_.image_weight);
          ceres::LossFunction *loss_function = NULL;
          loss_function = new ceres::CauchyLoss(10.0); // adopted from vins-mono
          ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_Image, cost_function, loss_function,
                                                                         vec, drop_set);
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
      }
    }

    marginalization_info->preMarginalize();
    marginalization_info->marginalize();
    if (lidar_marg_info)
    {
      lidar_marg_info = nullptr;
    }
    lidar_marg_info.reset(marginalization_info);
    lidar_marg_parameter_blocks = marginalization_info->getParameterBlocks();
  }

} // namespace cocolic
