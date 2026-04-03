/**
 * @file Arming.hpp
 * @brief ROS 2 Node for arming multiple drones using PX4 topic-based communication.
 * @author Arthur Astier (refactored for topic-based PX4 communication)
 */

#pragma once

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;

/**
 * @class Arming
 * @brief ROS 2 Node for arming multiple drones using PX4 topic-based communication.
 */
class Arming : public rclcpp::Node {
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;
    using VehicleStatus = px4_msgs::msg::VehicleStatus;

public:
    /**
     * @brief Constructor for the Arming class.
     */
    Arming();

    /**
     * @brief Arm the specified drone.
     * @param drone_idx Index of the drone to be armed.
     */
    void arm(std::size_t drone_idx);

    /**
     * @brief Set the specified drone to offboard mode.
     * @param drone_idx Index of the drone to be set to offboard mode.
     */
    void offboard_mode(std::size_t drone_idx);

private:
    /**
     * @brief Publish a vehicle command for the specified drone.
     * @param command Vehicle command to be published.
     * @param drone_idx Index of the drone.
     * @param param1 Command parameter 1.
     * @param param2 Command parameter 2.
     */
    void publish_vehicle_command(uint16_t command, std::size_t drone_idx, float param1 = 0.0, float param2 = 0.0);

    /**
     * @brief Callback for vehicle command acknowledgments.
     * @param msg Acknowledgment message.
     * @param drone_idx Index of the drone.
     */
    void command_ack_callback(const VehicleCommandAck::SharedPtr msg, std::size_t drone_idx);

    /**
     * @brief Callback for PX4 vehicle status updates.
     * @param msg Vehicle status message.
     * @param drone_idx Index of the drone.
     */
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg, std::size_t drone_idx);

private:
    std::vector<rclcpp::Publisher<VehicleCommand>::SharedPtr> vehicle_command_pubs_;
    std::vector<rclcpp::Subscription<VehicleCommandAck>::SharedPtr> command_ack_subs_;
    std::vector<rclcpp::Subscription<VehicleStatus>::SharedPtr> vehicle_status_subs_;
    std::vector<bool> is_armed_;
    std::vector<bool> is_offboard_;
    std::vector<uint8_t> target_system_ids_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::size_t nb_drones_;
    uint32_t command_id_;
};