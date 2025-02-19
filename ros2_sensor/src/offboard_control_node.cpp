#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/qos.hpp>  // QoS 관련 헤더 추가

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node"), offboard_setpoint_counter_(0), current_mode_(0)
    {
        // PX4의 QoS 설정 (BEST_EFFORT)
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        // VehicleStatus 구독 (현재 비행 모드 확인)
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

        // Publisher 설정
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // 타이머 설정 (100ms 주기)
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Offboard control node started.");
    }

private:
    int current_mode_;

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        current_mode_ = msg->nav_state;

        switch (current_mode_)
        {
        case 1:
            RCLCPP_INFO(this->get_logger(), "Current Mode: MANUAL");
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "Current Mode: ALTCTL (Altitude Control)");
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "Current Mode: POSCTL (Position Control)");
            break;
        case 14:
            RCLCPP_INFO(this->get_logger(), "Current Mode: OFFBOARD");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Current Mode: Unknown (%d)", current_mode_);
            break;
        }

        // 현재 모드가 OFFBOARD이면 POSCTL로 변경 후 다시 OFFBOARD로 전환
        if (current_mode_ == 14)
        {
            RCLCPP_WARN(this->get_logger(), "Already in OFFBOARD mode, switching to POSCTL first...");
            send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 2);
            rclcpp::sleep_for(1s); // 모드 변경 대기

            RCLCPP_INFO(this->get_logger(), "Switching back to OFFBOARD mode...");
            send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        }
    }

    void timer_callback()
    {
        if (offboard_setpoint_counter_ < 10)
        {
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            offboard_setpoint_counter_++;
        }
        else if (offboard_setpoint_counter_ == 10)
        {
            // 오프보드 모드로 변경
            send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // 기체 ARM
            send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(this->get_logger(), "Switched to OFFBOARD mode and ARMED the vehicle.");
            offboard_setpoint_counter_++;
        }
        else if (offboard_setpoint_counter_ < 100)
        {
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            offboard_setpoint_counter_++;
        }
        else
        {
            publish_offboard_control_mode();
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {0.0, 0.0, -5.0};  // 5m 상승
        msg.yaw = -3.14;  // 180도 회전
        trajectory_pub_->publish(msg);
    }

    void send_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int offboard_setpoint_counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
