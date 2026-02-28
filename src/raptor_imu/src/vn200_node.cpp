// Copyright (c) 2026 RAPTOR Project
// VN-200 IMU Node Entry Point

#include "raptor_imu/vn200_driver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto driver = std::make_shared<VN200Driver>();

    if (driver->init())
    {
        RCLCPP_ERROR(driver->get_logger(), "[IMU-ERROR] Failed to initialize VN-200. Shutting down.");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(driver->get_logger(), "[IMU-INFO] VN-200 IMU driver started.");

    while (rclcpp::ok())
    {
        try {
            driver->publishMessages();
            rclcpp::spin_some(driver);
        } catch (const rclcpp::exceptions::RCLError&) {
            break;  // Ctrl+C로 인한 shutdown → 루프 탈출
        }
    }

    RCLCPP_INFO(driver->get_logger(), "[IMU-INFO] VN-200 IMU driver shutting down.");
    rclcpp::shutdown();
    
    return 0;
}
