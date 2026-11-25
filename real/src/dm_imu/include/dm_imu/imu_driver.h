#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <initializer_list>
#include <fstream>
#include <array>
#include <serial/serial.h> // 确保安装了serial库
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include "dm_imu/bsp_crc.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "dm_imu/srv/calibrate_imu.hpp"

namespace dmbot_serial
{
  typedef struct
  {
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
    float roll;
    float pitch;
    float yaw;
  }IMU_Data;

  class DmImu 
  {
  public:
      // 构造函数和析构函数声明
      DmImu(rclcpp::Node::SharedPtr node);
      ~DmImu();

      // 公有方法声明
      void init_imu_serial();
      void get_imu_data_thread();
      void publish_imu_data();

  private:
      // 私有成员变量声明
      rclcpp::Node::SharedPtr node_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
      int imu_seial_baud;
      std::string imu_serial_port;
      serial::Serial serial_imu;
      std::thread rec_thread;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr euler2_pub_;

      std::unique_ptr<sensor_msgs::msg::Imu> imu_msgs;
      std::atomic<bool> stop_thread_{false};
      std::mutex data_mutex_;
      IMU_Data data{};
      bool have_euler_ = false;

      double roll_offset = 0.0, pitch_offset = 0.0, yaw_offset = 0.0;
      bool calibrate_on_startup_ = true;
      int calibration_samples_ = 100;
      int calibration_timeout_ms_ = 5000;
      bool publish_pose_ = false;
      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
      rclcpp::Service<dm_imu::srv::CalibrateIMU>::SharedPtr calibrate_service_;

    // 私有方法声明
    void enter_setting_mode();
    void turn_on_accel();
    void turn_on_gyro();
    void turn_on_euler();
    void turn_off_quat();
    void set_output_1000HZ();
    void save_imu_para();
    void exit_setting_mode();
    void restart_imu();
    void perform_calibration();
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters);
    void on_calibrate_imu(
        const std::shared_ptr<dm_imu::srv::CalibrateIMU::Request> request,
        std::shared_ptr<dm_imu::srv::CalibrateIMU::Response> response);
};

}
#endif