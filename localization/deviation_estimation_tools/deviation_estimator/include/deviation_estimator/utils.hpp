// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEVIATION_ESTIMATOR__UTILS_HPP_
#define DEVIATION_ESTIMATOR__UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tf2/transform_datatypes.h>

#include <fstream>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include "tf2/utils.h"

double double_round(const double x, const int n);

template <typename T>
double calculate_mean(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  double mean = 0;
  for (const T & t : v) {
    mean += t;
  }
  mean /= v.size();
  return mean;
}

template <typename T>
double calculate_std(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  const double mean = calculate_mean(v);
  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

template <typename T>
double calculate_std_mean_const(const std::vector<T> & v, const double mean)
{
  if (v.size() == 0) {
    return 0;
  }

  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

template <typename T>
inline rclcpp::Time extract_time(const T & msg)
{
  return rclcpp::Time(msg.header.stamp);
}

template <>
inline rclcpp::Time extract_time<tier4_debug_msgs::msg::Float64Stamped>(const tier4_debug_msgs::msg::Float64Stamped & msg)
{
  return rclcpp::Time(msg.stamp);
}

struct CompareMsgTimestamp
{
  template <typename T1>
  bool operator()(T1 const & t1, double const & t2) const
  {
    return extract_time<T1>(t1).seconds() < t2;
  }

  template <typename T2>
  bool operator()(double const & t1, T2 const & t2) const
  {
    return t1 < extract_time<T2>(t2).seconds();
  }

  template <typename T1, typename T2>
  bool operator()(T1 const & t1, T2 const & t2) const
  {
    return extract_time<T1>(t2).seconds() < extract_time<T2>(t2).seconds();
  }

  template <typename T1>
  bool operator()(T1 const & t1, rclcpp::Time const & t2) const
  {
    return extract_time<T1>(t1).seconds()< t2.seconds();
  }

  template <typename T2>
  bool operator()(rclcpp::Time const & t1, T2 const & t2) const
  {
    return t1.seconds() < extract_time<T2>(t2).seconds();
  }
};

geometry_msgs::msg::Vector3 interpolate_vector3_stamped(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & vec_list, const double time,
  const double tolerance_sec);

template <typename T>
std::vector<T> extract_sub_trajectory(
  const std::vector<T> & msg_list, const rclcpp::Time & t0, const rclcpp::Time & t1)
{
  const auto start_iter =
    std::upper_bound(msg_list.begin(), msg_list.end(), t0, CompareMsgTimestamp());
  const auto end_iter =
    std::lower_bound(msg_list.begin(), msg_list.end(), t1, CompareMsgTimestamp());
  std::vector<T> msg_list_sub(start_iter, end_iter - 1); // ToDo: may need to check if this iterators do not be inverted
  return msg_list_sub;
}

template <typename T>
struct Subtrajectory {
  geometry_msgs::msg::PoseStamped pose_start;
  geometry_msgs::msg::PoseStamped pose_end;
  std::vector<T> twist_list;
};

template <typename T>
Subtrajectory<T> extract_synchronized_pose_and_twist(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<T> & twist_list)
{
  const auto t1_pose = rclcpp::Time(pose_list.back().header.stamp);
  const auto t0_pose = rclcpp::Time(pose_list.front().header.stamp);
  if (t0_pose > t1_pose) {
    throw std::domain_error("Pose timestamp is inverted");
  }

  const std::vector<T> twist_sub_traj = extract_sub_trajectory(twist_list, t0_pose, t1_pose);
  const auto t0_twist = extract_time<T>(twist_sub_traj.front());
  const auto t1_twist = extract_time<T>(twist_sub_traj.back());

  // Should be t0_pose < t0_twist < t1_twist < t1_pose

  // /////////////////////////
  // // A necessary step for tuning timestamp difference
  // const auto pose_start_0 = *(pose_list.begin());
  // const auto pose_start_1 = *(pose_list.begin() + 1);
  // const auto t_pose_start_0 = extract_time(pose_start_0);
  // const auto t_pose_start_1 = extract_time(pose_start_1);
  // const double pose_start_rate = (t0_twist - t_pose_start_0).seconds() / (t_pose_start_1 - t_pose_start_0).seconds();

  // geometry_msgs::msg::PoseStamped pose_start;
  // pose_start.header.stamp = t0_twist;
  // pose_start.pose = tier4_autoware_utils::calcInterpolatedPose(pose_start_0, pose_start_1, pose_start_rate);

  // const auto pose_end_0 = *(pose_list.end() - 2);
  // const auto pose_end_1 = *(pose_list.end() - 1);
  // const auto t_pose_end_0 = extract_time(pose_end_0);
  // const auto t_pose_end_1 = extract_time(pose_end_1);
  // const double pose_end_rate = (t1_twist - t_pose_end_0).seconds() / (t_pose_end_1 - t_pose_end_0).seconds();

  // geometry_msgs::msg::PoseStamped pose_end;
  // pose_end.header.stamp = t1_twist;
  // pose_end.pose = tier4_autoware_utils::calcInterpolatedPose(pose_end_0, pose_end_1, pose_end_rate);
  // /////////////////////////
  const auto pose_start = pose_list.front();
  const auto pose_end = pose_list.back();
  
  const Subtrajectory<T> result {pose_start, pose_end, twist_sub_traj};
  return result;
}

template <typename T, typename U>
double norm_xy(const T p1, const U p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double clip_radian(const double rad);

void save_estimated_parameters(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset);

geometry_msgs::msg::Point integrate_position(
  const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list, const double coef_vx,
  const double yaw_init);

geometry_msgs::msg::Vector3 calculate_error_rpy(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias);

geometry_msgs::msg::Vector3 integrate_orientation(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias);

double get_mean_abs_vx(const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list);
double get_mean_abs_wz(const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list);

geometry_msgs::msg::Vector3 transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform);

inline void myFromMsg(const geometry_msgs::msg::Transform & in, tf2::Transform & out);

geometry_msgs::msg::TransformStamped inverse_transform(
  const geometry_msgs::msg::TransformStamped & transform);

#endif  // DEVIATION_ESTIMATOR__UTILS_HPP_
