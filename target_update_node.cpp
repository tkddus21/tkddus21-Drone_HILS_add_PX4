#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/qos.hpp>
#include <iostream>

using namespace std::chrono_literals;

class TargetUpdateNode : public rclcpp::Node
{
public:
    TargetUpdateNode() : Node("target_update_node")
    {
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            std::bind(&TargetUpdateNode::vehicle_position_callback, this, std::placeholders::_1));

        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        RCLCPP_INFO(this->get_logger(), "Target Update Node started. Waiting for user input...");
    }

    void update_target_position()
    {
        float dx, dy, dz;
        std::cout << "Enter target offset (dx dy dz): ";
        std::cin >> dx >> dy >> dz;

        float new_x = current_x_ + dx;
        float new_y = current_y_ + dy;
        float new_z = current_z_ + dz;

        publish_new_target(new_x, new_y, new_z);
    }

private:
    void vehicle_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
    }

    void publish_new_target(float x, float y, float z)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        trajectory_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "New target set: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    float current_x_, current_y_, current_z_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetUpdateNode>();

    while (rclcpp::ok())
    {
        node->update_target_position();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
