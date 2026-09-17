#ifndef ORB_POSE_PUBLISHER_HPP
#define ORB_POSE_PUBLISHER_HPP

#include <cstddef>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "MapPoint.h"

// ORB-SLAM uses the initial optical camera basis (x right, y down, z forward).
// Convert both the map and body bases to ROS REP-103 (x forward, y left, z up),
// yielding the rover/base pose with identity orientation at initialization.
class OrbPosePublisher
{
public:
  explicit OrbPosePublisher(rclcpp::Node * node)
  : node_(node)
  {
    node_->declare_parameter<std::string>("map_frame", "orb_map");
    node_->declare_parameter<int>("max_path_poses", 10000);
    node_->declare_parameter<bool>("planar_path", true);
    node_->declare_parameter<double>("map_z_offset", 0.12);
    frame_id_ = node_->get_parameter("map_frame").as_string();
    max_path_poses_ = static_cast<std::size_t>(
      std::max<int64_t>(1, node_->get_parameter("max_path_poses").as_int()));
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "orbslam3/pose", rclcpp::QoS(10));
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "orbslam3/path", rclcpp::QoS(1).transient_local());
    points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "orbslam3/map_points", rclcpp::QoS(1).transient_local());
    path_.header.frame_id = frame_id_;
  }

  void publish(const Sophus::SE3f & t_camera_world,
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<ORB_SLAM3::MapPoint *> & map_points = {},
    bool trajectory_ready = true)
  {
    const Sophus::SE3f t_world_camera = t_camera_world.inverse();
    Eigen::Matrix3f basis;
    basis << 0.0F, 0.0F, 1.0F,
            -1.0F, 0.0F, 0.0F,
             0.0F,-1.0F, 0.0F;

    const Eigen::Vector3f translation = basis * t_world_camera.translation();
    const Eigen::Matrix3f rotation =
      basis * t_world_camera.rotationMatrix() * basis.transpose();
    Eigen::Quaternionf quaternion(rotation);
    quaternion.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = node_->get_parameter("planar_path").as_bool() ? 0.0 : translation.z();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    // Inertial ORB-SLAM changes map scale and gravity direction during
    // initialization. Keeping those provisional poses creates vertical
    // spikes and discontinuities in an otherwise planar rover trajectory.
    if (trajectory_ready) {
      pose_pub_->publish(pose);
      path_.header.stamp = stamp;
      path_.poses.push_back(pose);
      if (path_.poses.size() > max_path_poses_) {
        path_.poses.erase(path_.poses.begin(),
          path_.poses.begin() + (path_.poses.size() - max_path_poses_));
      }
      path_pub_->publish(path_);
    }

    if (map_points.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    std::vector<Eigen::Vector3f> valid_points;
    valid_points.reserve(map_points.size());
    for (auto * point : map_points) {
      if (point == nullptr || point->isBad()) continue;
      Eigen::Vector3f p = basis * point->GetWorldPos();
      p.z() += static_cast<float>(node_->get_parameter("map_z_offset").as_double());
      if (p.allFinite()) valid_points.push_back(p);
    }
    modifier.resize(valid_points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & p : valid_points) {
      *x = p.x(); *y = p.y(); *z = p.z();
      ++x; ++y; ++z;
    }
    if (!valid_points.empty()) points_pub_->publish(cloud);
  }

private:
  rclcpp::Node * node_;
  std::string frame_id_;
  std::size_t max_path_poses_;
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

#endif
