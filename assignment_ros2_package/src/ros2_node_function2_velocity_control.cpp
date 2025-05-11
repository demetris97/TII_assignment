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
#include <limits>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

//ROS2 node class for velocity control
class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();
        
        //Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        intended_local_position_publisher_ = this->create_publisher<VehicleLocalPosition>("/controller/intended_local_position_input", 10);  // Use your desired topic name

        //Subscribers
        vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));
        
        // Intended waypoints are used only for plotting to compare them with actual trajectory.
        intended_waypoint_counter_ = 0;
        VehicleLocalPosition intended_waypoint_msg{};
        intended_waypoints_ = {
            {0.0f, 0.0f},
            {3000.0f, 0.0f},
            {3000.0f, 3000.0f},
            {0.0f, 3000.0f},
            {0.0f, 0.0f}
        };

        //Control state initialization 
        offboard_setpoint_counter_ = 0;
        all_waypoints_reached_ = false;

        current_velocity_ = {0.0f, 0.0f, -3.0f};  // Initial vertical descent
        current_yaw_ = 0.0f;
        segment_counter_ = 0;
        segment_length_ = 3000.0f;
        takeoff_done_ = false;

        //Timer callback running at 10Hz
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // PX4_MODE_OFFBOARD
                arm();
            }

            check_segment_distance(); //Manage waypoints transition
            publish_offboard_control_mode(); //Send offboard control mode message
            publish_trajectory_setpoint(); //Send velocity and yaw command 

            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };

        //Set up timer
        timer_ = this->create_wall_timer(100ms, timer_callback);

        // Publish the intended waypoint. For visualization and comparison purposes
        intended_waypoint_msg.x = intended_waypoints_[intended_waypoint_counter_][0];
        intended_waypoint_msg.y = intended_waypoints_[intended_waypoint_counter_][1];
        intended_waypoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        intended_local_position_publisher_->publish(intended_waypoint_msg);
        intended_waypoint_counter_++;
    }

    //Command functions
    void arm();
    void disarm();
    void land();
    //Initialization of variables
    float current_x_;
    float current_y_;
    float current_z_;

    float current_vx_;
    float current_vy_;
    float current_vz_;

private:
    rclcpp::TimerBase::SharedPtr timer_;
    //Publishers and Subscribers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleLocalPosition>::SharedPtr intended_local_position_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    std::vector<std::array<float, 2>> intended_waypoints_;

    uint64_t offboard_setpoint_counter_;
    bool all_waypoints_reached_;

    // Dynamic control state
    float segment_start_x_;
    float segment_start_y_;
    float segment_start_z_;
    std::array<float, 3> current_velocity_; //Current velocity command
    float current_yaw_; //Yaw command
    size_t segment_counter_; //Which segment of the square
    int intended_waypoint_counter_; 
    float segment_length_;
    bool takeoff_done_;

    //Helper functions
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void check_segment_distance();
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

//Send land command
void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command sent");
}

//Send control mode for velocity control
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

//Send current velocity setpoint
void OffboardControl::publish_trajectory_setpoint()
{
    if (all_waypoints_reached_) return;

    TrajectorySetpoint msg{};
    msg.position = {
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()
    };

    msg.velocity = {current_velocity_[0], current_velocity_[1], current_velocity_[2]};
    msg.yaw = current_yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    trajectory_setpoint_publisher_->publish(msg);
}

//Set up and publish vehicle command
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

//Callback to update the current position and velocity of the drone based on sensor feedback
void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
    current_vx_ = msg->vx;
    current_vy_ = msg->vy;
    current_vz_ = msg->vz;
}

//Check how far the drone traveled in current segment and swithc to the next segment if 3km have been covered
void OffboardControl::check_segment_distance()
{
    if (all_waypoints_reached_) return;

    //Handle takeoff logic: ascend to -50 meters altitude (NED)
    if (!takeoff_done_) {
        float dz = current_z_ - (-50.0f);  // Assuming target altitude is -50m (NED)
        if (std::fabs(dz) < 1.0f) {
            takeoff_done_ = true;
            segment_start_x_ = current_x_;
            segment_start_y_ = current_y_;
            segment_start_z_ = current_z_;

            current_velocity_ = {50.0f, 0.0f, 0.0f};
            current_yaw_ = 0.0f;

            RCLCPP_INFO(this->get_logger(), "Takeoff complete. Starting square path...");
        } else {
            // Still taking off
            current_velocity_ = {0.0f, 0.0f, -15.0f};
            current_yaw_ = 0.0f;
        }
        return;
    }

    //At the second waypoint (the 1st was 50m altitude after takeoff) we want the VTOL to have 500 height
    if (current_z_ < -500.0f) {
        current_velocity_[2] = 0.0f; // Stop increasing the altitude (NED: down is positive, up is negative)
    } else {
        current_velocity_[2] = -15.0f;  // Continue increasing the altitude (NED: down is poitive, up is negative)
    }

    // Measure distance traveled in current segment
    float dx = current_x_ - segment_start_x_;
    float dy = current_y_ - segment_start_y_;
    float distance = std::sqrt(dx * dx + dy * dy);

    float velocity = std::sqrt(std::pow(current_vx_, 2) + std::pow(current_vy_, 2) + std::pow(current_vz_, 2)); 
    RCLCPP_INFO(this->get_logger(), "Segment %zu | Distance: %.2f | Velocity: %.2f", segment_counter_, distance, velocity);

    if (distance >= segment_length_) {
        segment_counter_++;

        //Square path complete
        if (segment_counter_ >= 4) {
            all_waypoints_reached_ = true;
            current_velocity_ = {0.0f, 0.0f, 0.0f};
            RCLCPP_INFO(this->get_logger(), "Completed square path.");
            
            // Publish the intended waypoint. For visualization and comparison purposes
            VehicleLocalPosition intended_waypoint_msg{};
            intended_waypoint_msg.x = intended_waypoints_[intended_waypoint_counter_][0];
            intended_waypoint_msg.y = intended_waypoints_[intended_waypoint_counter_][1];
            intended_waypoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            intended_local_position_publisher_->publish(intended_waypoint_msg);
            intended_waypoint_counter_++;

            land();
            rclcpp::sleep_for(10s);
            disarm();
            return;
        }

        // Rotate 90° anticlockwise and wrap the current_yaw in the range [-pi, pi]
        current_yaw_ += M_PI_2; // M_PI_2 is a predefined constant fo pi/2
        if (current_yaw_ > M_PI) {
            current_yaw_ -= 2 * M_PI;
        }

        // Update velocity direction
        current_velocity_[0] = 50.0f * std::cos(current_yaw_);
        current_velocity_[1] = 50.0f * std::sin(current_yaw_);
        current_velocity_[2] = 0.0f;

        segment_start_x_ = current_x_;
        segment_start_y_ = current_y_;
        segment_start_z_ = current_z_;

        RCLCPP_INFO(this->get_logger(), "Starting segment %zu with velocity (%.2f, %.2f) and yaw %.2f",
                    segment_counter_, current_velocity_[0], current_velocity_[1], current_yaw_);
        
        // Publish the intended waypoint. For visualization and comparison purposes
        VehicleLocalPosition intended_waypoint_msg{};
        intended_waypoint_msg.x = intended_waypoints_[intended_waypoint_counter_][0];
        intended_waypoint_msg.y = intended_waypoints_[intended_waypoint_counter_][1];
        intended_waypoint_msg.z = std::numeric_limits<float>::quiet_NaN();
        intended_waypoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        intended_local_position_publisher_->publish(intended_waypoint_msg);
        RCLCPP_INFO(this->get_logger(), "Published simulated waypoint: (%.2f, %.2f)",
                    intended_waypoint_msg.x, intended_waypoint_msg.y);
        intended_waypoint_counter_++;
 
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
