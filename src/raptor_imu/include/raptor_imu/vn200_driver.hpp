// Copyright (c) 2026 RAPTOR Project
// VN-200 IMU Driver for VIO (Visual-Inertial Odometry)
// Based on VectorNav SDK v1.2.0

#ifndef RAPTOR_IMU__VN200_DRIVER_HPP_
#define RAPTOR_IMU__VN200_DRIVER_HPP_

#include <memory>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// VN SDK
#include "vectornav/Config.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/TemplateLibrary/Matrix.hpp"

// RAPTOR 커스텀 메시지
#include "raptor_msgs/msg/imu_group.hpp"
#include "raptor_msgs/msg/attitude_group.hpp"
#include "raptor_msgs/msg/time_group.hpp"
#include "raptor_msgs/srv/transaction.hpp"

// ─── 유틸리티 변환 함수 ─── //

inline geometry_msgs::msg::Vector3 toVector3(const VN::Vec3f& input)
{
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = input[0];
    lhs.y = input[1];
    lhs.z = input[2];
    return lhs;
}

inline geometry_msgs::msg::Vector3 toVector3(const VN::Ypr& input)
{
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = input.yaw;
    lhs.y = input.pitch;
    lhs.z = input.roll;
    return lhs;
}

inline geometry_msgs::msg::Quaternion toQuaternion(const VN::Quat& input)
{
    geometry_msgs::msg::Quaternion lhs;
    lhs.x = input.vector[0];
    lhs.y = input.vector[1];
    lhs.z = input.vector[2];
    lhs.w = input.scalar;
    return lhs;
}

// ─── VN200Driver 클래스 ─── //

class VN200Driver : public rclcpp::Node {
public:
    VN200Driver();

    /// @brief 센서 연결 및 설정 초기화
    /// @return true: 에러 발생, false: 성공
    bool init();

    /// @brief 센서에서 측정값을 읽고 ROS2 토픽으로 publish
    void publishMessages();

private:
    /// @brief VN-200 센서에 연결
    bool connect(const std::string port, const int requestedBaudRate);

    /// @brief 센서 레지스터 설정 (Binary Output, SyncControl 등)
    bool configureSensor();

    /// @brief VN SDK Transaction 서비스 콜백
    void vnTransaction(
        const std::shared_ptr<raptor_msgs::srv::Transaction::Request> request,
        const std::shared_ptr<raptor_msgs::srv::Transaction::Response> response);

    // ─── Publish 함수 ─── //
    void publishStdImu(const VN::CompositeData* cd, const rclcpp::Time& rosTime);
    void publishImuGroup(const VN::CompositeData* cd, const rclcpp::Time& rosTime);
    void publishAttitudeGroup(const VN::CompositeData* cd, const rclcpp::Time& rosTime);
    void publishTimeGroup(const VN::CompositeData* cd, const rclcpp::Time& rosTime);

    // ─── Publishers ─── //
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubStdImu_;
    rclcpp::Publisher<raptor_msgs::msg::ImuGroup>::SharedPtr pubImuGroup_;
    rclcpp::Publisher<raptor_msgs::msg::AttitudeGroup>::SharedPtr pubAttitudeGroup_;
    rclcpp::Publisher<raptor_msgs::msg::TimeGroup>::SharedPtr pubTimeGroup_;

    // ─── Service ─── //
    rclcpp::Service<raptor_msgs::srv::Transaction>::SharedPtr srvTransaction_;

    // ─── VN Sensor ─── //
    VN::Sensor vs_;
};

#endif  // RAPTOR_IMU__VN200_DRIVER_HPP_
