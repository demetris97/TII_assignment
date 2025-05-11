#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// Main class for control
class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // Set QoS with best effort
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Subscriber
        vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

        offboard_setpoint_counter_ = 0;
        current_waypoint_index_ = 0; // Index of the waypoint that we want to reach
        all_waypoints_reached_ = false;


        // Waypoints: {x, y, z}
        waypoints_ = {
            {0.0f, 0.0f, -50.0f, 0.0f},       // 1. Takeoff to 50m, facing south
            {3000.0f, 0.0f, -500.0f, 0.0f},     // 2. Move forward (westward), same yaw
            {3000.0f, 3000.0f, -500.0f, 1.57f},      // 3. Turn right and go north, yaw east
            {0.0f, 3000.0f, -500.0f, 3.14f},        // 4. Turn right and go east, yaw north
            {0.0f, 0.0f, -500.0f, -1.57f},        // 5. Complete square, face west
            {0.0f, 0.0f, 0.0f, -1.57f}        // 6. decrease altitude
        };


        // Timer
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Set to offboard
                arm();
            }

            check_waypoint_reached(); // Checks the distance between the drone and the target point/waypoint. Sets new waypoints if previous has been reached. Land & disarm if all waypoints reached.
            // Send contorl messages
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            // Increase the counter up to 10. Just needed to arm the drone after 1 second.
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm(); // Arm the drone
    void disarm(); // Disarm the drone
    void land(); //Land the drone

    // Initialization of current position variables
    float current_x_;
    float current_y_;
    float current_z_;

private:
    rclcpp::TimerBase::SharedPtr timer_; // Timer object

    //Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    //Subscriber
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    uint64_t offboard_setpoint_counter_; //Counter to initiate arming after 1 second
    std::vector<std::array<float, 4>> waypoints_; // {x,y,z,yaw}
    size_t current_waypoint_index_; //Current waypoint index
    bool all_waypoints_reached_; //Flag that indicates mission completion

    //Helper functions
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);

    void check_waypoint_reached();
};

//Send arm command
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

//Send disarm command
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

//Send Land command
void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command sent");
}
/* Included for transition to Fixed-wing mode of the VTOL. Many attemps have been done but something was not leting the drone to switch to Fixed-wing mode.
void OffboardControl::vtol_takeoff()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_VTOL_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 47.39773615102264f,8.547064163497595f,15.0f);
    RCLCPP_INFO(this->get_logger(), "VTOL takeoff command sent");

}
*/

//Publish control mode message
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

//Publish next target waypoint as trajectory setpoint
void OffboardControl::publish_trajectory_setpoint()
{
    if (all_waypoints_reached_ || current_waypoint_index_ >= waypoints_.size()) {
        return;
    }

    TrajectorySetpoint msg{};
    auto target = waypoints_[current_waypoint_index_];
    msg.position = {target[0], target[1], target[2]};
    msg.yaw = target[3];
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

//Publish general vehicle command
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

//Callback to update the drone's current position from sensor feedback
void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
}

//Check if current waypoint is reached and update to next
void OffboardControl::check_waypoint_reached()
{
    if (all_waypoints_reached_ || current_waypoint_index_ >= waypoints_.size()) return;

    auto target = waypoints_[current_waypoint_index_];
    float dx = current_x_ - target[0];
    float dy = current_y_ - target[1];
    float dz = current_z_ - target[2];
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    //Log current and target position
    RCLCPP_INFO(this->get_logger(), "Current Pos: (%.2f, %.2f) | Target %zu -> (%.2f, %.2f) | Distance: %.2f",
                current_x_, current_y_,
                current_waypoint_index_,
                target[0], target[1], 
                distance);

    //If close enough to target waypoint, switch to the next waypoint
    if (distance < 1.0f) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_);
        current_waypoint_index_++;

        //Checks if the autonomous mission is completed in order to land and disarm
        if (current_waypoint_index_ >= waypoints_.size()) {
            all_waypoints_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "All waypoints reached.");

            land(); // Land the drone
            rclcpp::sleep_for(15s);
            disarm(); // Disarm the drone
        }
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
