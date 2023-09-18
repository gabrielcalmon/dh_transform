#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dh_transform/srv/angles_update.hpp"

// This classes simulates a group of joint encoder's, publishing the angular postion
class FakeEncoderPublisher : public rclcpp::Node {
public:
    FakeEncoderPublisher(int n_joints, double j_value) : 
    Node("fake_encoder_publisher"), num_joints(n_joints), default_joints_value(j_value)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_angles", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&FakeEncoderPublisher::publish_sensor_values, this));

        fake_deg_values = std::vector<double>(num_joints, default_joints_value);

        // Crie um serviço para receber atualizações do vetor de dados
        update_angles_service_ = this->create_service<dh_transform::srv::AnglesUpdate>(
            "update_angles", std::bind(&FakeEncoderPublisher::updateAnglesCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Service used to update joint angles in runtime
    void updateAnglesCallback(
        const std::shared_ptr<dh_transform::srv::AnglesUpdate::Request> request,
        std::shared_ptr<dh_transform::srv::AnglesUpdate::Response> response)
    {
        fake_deg_values = request->angles;
        RCLCPP_INFO(this->get_logger(), "Received array of angles: ");
        for (const auto& value : fake_deg_values) {
            RCLCPP_INFO(this->get_logger(), "%.2f", value);
        }

        response->success = true;
    }

private:
    void publish_sensor_values() {
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        joint_state_msg->name.resize(num_joints);
        joint_state_msg->position.resize(num_joints);

        for (int i = 0; i < num_joints; i++) {
            joint_state_msg->name[i] = "joint" + std::to_string(i + 1);
            joint_state_msg->position[i] = fake_deg_values[i];
        }

        publisher_->publish(std::move(joint_state_msg));
    }

    const int num_joints;
    std::vector<double> fake_deg_values;
    double default_joints_value;     // initial angular position (in degrees) 
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<dh_transform::srv::AnglesUpdate>::SharedPtr update_angles_service_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeEncoderPublisher>(3, 0);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}