// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include "gnss_ins_msg_buffer/gnss_ins_msg_buffer.h"

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

CartesianConv::CartesianConv()
    : Node("SensorSubscriber") {
  std::cout.precision(20);

//  vehicle_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
//          "/vehicle/status/twist_with_covariance",
//          10);
//
//  vehicle_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
//          "gyro_twist_with_covariance",
//          10);
  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
          "/sensing/imu/imu_data",
          10);
//  pvt_msg_publisher = this->create_publisher<ublox_msgs::msg::NavPVT>(
//      "/navpvt",
//      10);
  navsat_fix_msg_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/applanix/lvx_client/gnss/fix",
      10);
  autoware_msg_publisher = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
    "/autoware_orientation",
    10);
    clock_msg_publisher = this->create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock",
            100);
  msg_49_sub_ = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
          "/lvx_client/gsof/ins_solution_49", 10,
          std::bind(&CartesianConv::msg_49_callback, this, std::placeholders::_1));
  msg_50_sub_ = this->create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
            "/lvx_client/gsof/ins_solution_rms_50", 10,
            std::bind(&CartesianConv::msg_50_callback, this, std::placeholders::_1));
    vehicle_time_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&CartesianConv::vehicle_time_callback, this, std::placeholders::_1));

}


void CartesianConv::vehicle_time_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr vehicle_msg){
    clock_msg.clock = vehicle_msg->header.stamp;
}


void CartesianConv::msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr Msg_49) {

    tf2::Quaternion quaternion, ins_corrected_quat;
    double roll = Msg_49->roll * deg2rad;
    double pitch = Msg_49->pitch * deg2rad;
    double yaw = Msg_49->heading * deg2rad;
    quaternion.setRPY(roll, pitch, yaw);

    tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_, ins_corrected_rot_, applanix2ros;
    ins_rot_matrix.setRotation(quaternion);

    ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
    applanix2ros = tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    ins_rot_ = ENU2NED * ins_rot_matrix;

    ins_corrected_rot_ = ins_rot_ * applanix2ros;
    ins_corrected_rot_.getRotation(ins_corrected_quat);


    autoware_orientation_msg.header.frame_id = "base_link";
    autoware_orientation_msg.header.stamp = clock_msg.clock;
    autoware_orientation_msg.orientation.orientation.x = ins_corrected_quat.getX();
    autoware_orientation_msg.orientation.orientation.y = ins_corrected_quat.getY();
    autoware_orientation_msg.orientation.orientation.z = ins_corrected_quat.getZ();
    autoware_orientation_msg.orientation.orientation.w = ins_corrected_quat.getW();
    autoware_msg_publisher->publish(autoware_orientation_msg);


    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = clock_msg.clock;
    imu_msg.orientation.x = ins_corrected_quat.getX();
    imu_msg.orientation.y = ins_corrected_quat.getY();
    imu_msg.orientation.z = ins_corrected_quat.getZ();
    imu_msg.orientation.w = ins_corrected_quat.getW();

    imu_msg.angular_velocity.z = - Msg_49->ang_rate_down * deg2rad;
    imu_msg.linear_acceleration.x = Msg_49->acc_trans;
    imu_msg.linear_acceleration.y = Msg_49->acc_long;
    imu_msg.linear_acceleration.z = - Msg_49->acc_down;

    imu_publisher->publish(imu_msg);


    nav_sat_fix_msg.header.frame_id = "gnss";
    nav_sat_fix_msg.header.stamp = clock_msg.clock;

    nav_sat_fix_msg.latitude = Msg_49->lla.latitude;
    nav_sat_fix_msg.longitude = Msg_49->lla.longitude;
    nav_sat_fix_msg.altitude = Msg_49->lla.altitude;

    navsat_fix_msg_publisher->publish(nav_sat_fix_msg);

    // clock_msg part
    clock_msg_publisher->publish(clock_msg);

//    vehicle_twist_.header.frame_id = "base_link";
//    vehicle_twist_.header.stamp = Msg_49->header.stamp;
//    vehicle_twist_.twist.twist.linear.x = Msg_49->total_speed;
//    vehicle_twist_.twist.twist.angular.z = - Msg_49->ang_rate_down * deg2rad;
//    vehicle_twist_publisher->publish(vehicle_twist_);

}
void CartesianConv::msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr Msg_50) {

    vehicle_twist_.twist.covariance[0]  = std::pow(Msg_50->vel_rms_error.east, 2);
    vehicle_twist_.twist.covariance[7]  = std::pow(Msg_50->vel_rms_error.north, 2);
    vehicle_twist_.twist.covariance[14] = std::pow(Msg_50->vel_rms_error.down, 2);

    vehicle_twist_.twist.covariance[21] = 1000;
    vehicle_twist_.twist.covariance[28] = 1000;
    vehicle_twist_.twist.covariance[35] = 1000;

    autoware_orientation_msg.orientation.rmse_rotation_x = pow(Msg_50->attitude_rms_error_roll, 2);
    autoware_orientation_msg.orientation.rmse_rotation_y = pow(Msg_50->attitude_rms_error_pitch, 2);
    autoware_orientation_msg.orientation.rmse_rotation_z = pow(Msg_50->attitude_rms_error_heading, 2);

    nav_sat_fix_msg.position_covariance[0] = std::pow(Msg_50->pos_rms_error.east, 2);
    nav_sat_fix_msg.position_covariance[1] = std::pow(Msg_50->pos_rms_error.north, 2);
    nav_sat_fix_msg.position_covariance[2] = std::pow(Msg_50->pos_rms_error.down, 2);
    nav_sat_fix_msg.position_covariance[3] = std::pow(Msg_50->attitude_rms_error_roll * deg2rad, 2);
    nav_sat_fix_msg.position_covariance[4] = std::pow(Msg_50->attitude_rms_error_pitch * deg2rad, 2);
    nav_sat_fix_msg.position_covariance[5] = std::pow(Msg_50->attitude_rms_error_heading * deg2rad, 2);
    nav_sat_fix_msg.position_covariance_type = 1;

    imu_msg.angular_velocity_covariance[0]  = std::pow(Msg_50->vel_rms_error.east, 2);
    imu_msg.angular_velocity_covariance[7]  = std::pow(Msg_50->vel_rms_error.north, 2);
    imu_msg.angular_velocity_covariance[14] = std::pow(Msg_50->vel_rms_error.down, 2);

    imu_msg.angular_velocity_covariance[21] = 1000;
    imu_msg.angular_velocity_covariance[28] = 1000;
    imu_msg.angular_velocity_covariance[35] = 1000;
}



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}
