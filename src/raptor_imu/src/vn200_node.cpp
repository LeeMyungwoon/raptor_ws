// Copyright (c) 2026 RAPTOR Project
// VN-200 IMU Node Entry Point

#include "raptor_imu/vn200_driver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto driver = std::make_shared<VN200Driver>();

    if (driver->init())
    {
        RCLCPP_ERROR(driver->get_logger(), "Failed to initialize VN-200. Shutting down.");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(driver->get_logger(), "VN-200 IMU driver started.");

    while (rclcpp::ok())
    {
        driver->publishMessages();
        rclcpp::spin_some(driver);
    }

    rclcpp::shutdown();
    return 0;
}
