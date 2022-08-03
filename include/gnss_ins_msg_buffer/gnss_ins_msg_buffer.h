// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_
#define GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"


class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();
//  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vehicle_twist_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
//  rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr pvt_msg_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_msg_publisher;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_msg_publisher;
  rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr autoware_msg_publisher;

  double deg2rad = M_PI / 180;
  double rad2deg = 180 / M_PI;
  sensor_msgs::msg::NavSatFix  nav_sat_fix_msg;
  rosgraph_msgs::msg::Clock clock_msg;
  sensor_msgs::msg::Imu imu_msg;
  autoware_sensing_msgs::msg::GnssInsOrientationStamped autoware_orientation_msg;

  rclcpp::Subscription<applanix_msgs::msg::NavigationSolutionGsof49>::SharedPtr msg_49_sub_;
  rclcpp::Subscription<applanix_msgs::msg::NavigationPerformanceGsof50>::SharedPtr msg_50_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_time_sub_;

  void msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg);
    void msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg);
    void vehicle_time_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

    autoware_auto_vehicle_msgs::msg::VelocityReport vehicle_status;
    geometry_msgs::msg::TwistWithCovarianceStamped vehicle_twist_;

};

#endif  // GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_
