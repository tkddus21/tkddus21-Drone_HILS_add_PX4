#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

class YawControlNode : public rclcpp::Node {
public:
    YawControlNode() : Node("yaw_control_node") {
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        // 라이다 센서 구독
        left_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/iris/left_lidar_plugin/out", qos_profile,
            std::bind(&YawControlNode::left_lidar_callback, this, std::placeholders::_1));

        right_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/iris/right_lidar_plugin/out", qos_profile,
            std::bind(&YawControlNode::right_lidar_callback, this, std::placeholders::_1));

        // 현재 위치 구독
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            std::bind(&YawControlNode::local_position_callback, this, std::placeholders::_1));

        // 퍼블리셔 설정
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        // 제어 타이머 (100ms 주기)
        timer_ = this->create_wall_timer(100ms, std::bind(&YawControlNode::control_callback, this));

        RCLCPP_INFO(this->get_logger(), "Yaw Control Node started");
    }

private:
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_position_ = {msg->x, msg->y, msg->z};
        current_yaw_ = msg->heading;
    }

    void left_lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->data.size() >= 16) {
            left_distance_ = *reinterpret_cast<const float*>(&msg->data[0]);
        }
    }

    void right_lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->data.size() >= 16) {
            right_distance_ = *reinterpret_cast<const float*>(&msg->data[0]);
        }
    }

    void control_callback() {
        float distance_diff = left_distance_ - right_distance_;
        
        if (std::abs(distance_diff) > DISTANCE_THRESHOLD) {
            current_yaw_ += (distance_diff > 0) ? YAW_RATE : -YAW_RATE;
            
            px4_msgs::msg::TrajectorySetpoint msg{};
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            msg.position = current_position_;
            msg.yaw = current_yaw_;
            trajectory_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), 
                       "Adjusting yaw: %.2f (L: %.2f, R: %.2f, diff: %.2f)",
                       current_yaw_, left_distance_, right_distance_, distance_diff);
        }
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_lidar_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<float, 3> current_position_;
    float current_yaw_;
    float left_distance_;
    float right_distance_;
    
    const float YAW_RATE = 0.025;
    const float DISTANCE_THRESHOLD = 0.05;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawControlNode>());
    rclcpp::shutdown();
    return 0;
} 