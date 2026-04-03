#include "Arming.hpp"

Arming::Arming() : Node("arming"), command_id_(0) {
    // Retrieve the number of drones from the parameter server
    this->declare_parameter<int>("nb_drones");
    nb_drones_ = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());

    // Initialize vectors
    vehicle_command_pubs_.reserve(nb_drones_);
    command_ack_subs_.reserve(nb_drones_);
    vehicle_status_subs_.reserve(nb_drones_);
    is_armed_.assign(nb_drones_, false);
    is_offboard_.assign(nb_drones_, false);
    target_system_ids_.assign(nb_drones_, 0);

    // QoS profile for PX4 communication
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // Create publishers and subscribers for each drone
    for (std::size_t i = 0; i < nb_drones_; ++i) {
        std::string namespace_prefix = "/px4_" + std::to_string(i + 1);
        
        // Create vehicle command publisher
        std::string cmd_topic = namespace_prefix + "/fmu/in/vehicle_command";
        vehicle_command_pubs_.push_back(
            this->create_publisher<VehicleCommand>(cmd_topic, qos)
        );

        // Create command acknowledgment subscriber
        std::string ack_topic = namespace_prefix + "/fmu/out/vehicle_command_ack";
        command_ack_subs_.push_back(
            this->create_subscription<VehicleCommandAck>(
                ack_topic, qos,
                [this, i](const VehicleCommandAck::SharedPtr msg) {
                    command_ack_callback(msg, i);
                }
            )
        );

        // Subscribe to vehicle status to discover PX4 MAV_SYS_ID per instance.
        std::string status_topic = namespace_prefix + "/fmu/out/vehicle_status_v1";
        vehicle_status_subs_.push_back(
            this->create_subscription<VehicleStatus>(
                status_topic, qos,
                [this, i](const VehicleStatus::SharedPtr msg) {
                    vehicle_status_callback(msg, i);
                }
            )
        );

        RCLCPP_INFO(this->get_logger(), "Initialized arming for drone n°%zu", i + 1);
    }

    // Create timer to periodically send arm and offboard commands
    timer_ = this->create_wall_timer(1s, [this]() {
        // Check if all drones are armed and in offboard mode
        bool all_armed = std::all_of(is_armed_.begin(), is_armed_.end(), 
                                      [](bool v) { return v; });
        bool all_offboard = std::all_of(is_offboard_.begin(), is_offboard_.end(), 
                                         [](bool v) { return v; });

        // Shutdown if all drones are ready
        if (all_armed && all_offboard) {
            RCLCPP_INFO(this->get_logger(), "All drones armed and in offboard mode. Shutting down arming node.");
            rclcpp::shutdown();
            return;
        }

        // Send commands to drones that are not ready
        for (std::size_t i = 0; i < nb_drones_; ++i) {
            if (!is_offboard_[i]) {
                offboard_mode(i);
            }
            if (!is_armed_[i]) {
                arm(i);
            }
        }
    });
}


/**
 * @brief Arm the specified drone.
 * @param drone_idx Index of the drone to be armed.
 */
void Arming::arm(std::size_t drone_idx) {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, drone_idx, 1.0);
}

/**
 * @brief Set the specified drone to offboard mode.
 * @param drone_idx Index of the drone to be set to offboard mode.
 */
void Arming::offboard_mode(std::size_t drone_idx) {
    // param1 = 1 (custom mode enabled), param2 = 6 (offboard mode)
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, drone_idx, 1.0, 6.0);
}

/**
 * @brief Publish a vehicle command for the specified drone.
 * @param command Vehicle command to be published.
 * @param drone_idx Index of the drone.
 * @param param1 Command parameter 1.
 * @param param2 Command parameter 2.
 */
void Arming::publish_vehicle_command(uint16_t command, std::size_t drone_idx, 
                                      float param1, float param2) {
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    const auto detected_target = target_system_ids_[drone_idx];
    if (detected_target == 0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "PX4 system ID not detected yet for drone n°%zu. Using fallback target_system=%zu",
            drone_idx + 1, drone_idx + 1);
    }
    msg.target_system = detected_target != 0 ? detected_target : static_cast<uint8_t>(drone_idx + 1);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.confirmation = 0;

    vehicle_command_pubs_[drone_idx]->publish(msg);
}

/**
 * @brief Callback for vehicle command acknowledgments.
 * @param msg Acknowledgment message.
 * @param drone_idx Index of the drone.
 */
void Arming::command_ack_callback(const VehicleCommandAck::SharedPtr msg, std::size_t drone_idx) {
    // Check if command was successful (result == 0 means success)
    if (msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
        if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM && !is_armed_[drone_idx]) {
            is_armed_[drone_idx] = true;
            RCLCPP_INFO(this->get_logger(), "✓ Drone n°%zu is armed!", drone_idx + 1);
        } else if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE && !is_offboard_[drone_idx]) {
            is_offboard_[drone_idx] = true;
            RCLCPP_INFO(this->get_logger(), "✓ Drone n°%zu is in offboard mode!", drone_idx + 1);
        }
    } else if (msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
        RCLCPP_WARN(this->get_logger(), "Command %u for drone n°%zu failed with result: %u",
                    msg->command, drone_idx + 1, msg->result);
    }
}

/**
 * @brief Callback for vehicle status updates used to detect PX4 MAV_SYS_ID.
 * @param msg Vehicle status message.
 * @param drone_idx Index of the drone.
 */
void Arming::vehicle_status_callback(const VehicleStatus::SharedPtr msg, std::size_t drone_idx) {
    const uint8_t detected_system_id = msg->system_id;
    if (detected_system_id == 0) {
        return;
    }

    if (target_system_ids_[drone_idx] != detected_system_id) {
        target_system_ids_[drone_idx] = detected_system_id;
        RCLCPP_INFO(this->get_logger(), "Detected system ID for drone n°%zu: %u",
                    drone_idx + 1, static_cast<unsigned int>(detected_system_id));
    }
}

/**
 * @brief Main function to start the arming node.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Exit code.
 */
int main(int argc, char *argv[]) {
    std::cout << "Starting arming drones..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arming>());
    rclcpp::shutdown();
    return 0;
}
