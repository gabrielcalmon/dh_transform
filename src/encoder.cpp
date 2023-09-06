#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/* This classes simulates a group of joint encoder's, publishing the angular postion */
class FakeEncoderPublisher : public rclcpp::Node {
public:
    FakeEncoderPublisher(int n_joints, double j_value) : 
    Node("fake_encoder_publisher"), num_joints(n_joints), default_joints_value(j_value)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_angles", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&FakeEncoderPublisher::publish_sensor_values, this));

        fake_deg_values = std::vector<double>(num_joints, default_joints_value);
        
        // update_joint_values();
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

    void update_joint_values(){
        std::cout << "Type " << num_joints << " angular values separated by space:" << std::endl;
        for (int i = 0; i < num_joints; i++) {
            std::cin >> fake_deg_values[i];
        }
    }

    const int num_joints;
    std::vector<double> fake_deg_values;
    double default_joints_value;     // initial angular position (in degrees) 
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeEncoderPublisher>(5, 0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}