// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace radar_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(rclcpp::Node::SharedPtr pnh) : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
    auto declare_get = [&](const std::string& name, auto default_val) {
        if (!pnh->has_parameter(name))
            pnh->declare_parameter(name, default_val);
        return pnh->get_parameter(name).get_value<decltype(default_val)>();
    };
    keyframe_delta_trans = declare_get("keyframe_delta_trans", 2.0);
    keyframe_delta_angle = declare_get("keyframe_delta_angle", 2.0);
    // keyframe_delta_time = declare_get("keyframe_delta_time", 2.0);
    keyframe_min_size = declare_get("keyframe_min_size", 1000);

    accum_distance = 0.0;
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool decide(const Eigen::Isometry3d& pose, const rclcpp::Time& stamp) {
    // first frame is always registered to the graph
    if(is_first) {
      is_first = false;
      prev_keypose = pose;
      prev_keytime = stamp.seconds();
      return true;
    }
    
    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle();
    // double dt = stamp.seconds()() - prev_keytime;

    // too close to the previous frame
    if((dx < keyframe_delta_trans && da < keyframe_delta_angle)) {
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    prev_keytime = stamp.seconds();

    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const {
    return accum_distance;
  }

private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //
  std::size_t keyframe_min_size;  //

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
  double prev_keytime;
};

}  // namespace radar_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
