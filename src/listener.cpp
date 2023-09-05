#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Eigen::MatrixXd;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher")//, count_(0)
    {
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // timer_ = this->create_wall_timer(
        // 500ms, std::bind(&MinimalPublisher::timer_callback, this));

        
        this->declare_parameter("number_of_joints", 0);
        number_of_joints = get_parameter("number_of_joints").as_int();
        RCLCPP_INFO(get_logger(), "Number of joints: %d",
                static_cast<int>(number_of_joints));

        this->declare_parameter("dh_params", std::vector<double>(number_of_joints+1,-1));

        dh_vector = get_parameter("dh_params").as_double_array();

        if((number_of_joints+1)%4 != 0){    // It is expected that the DH params come in multiples of 4
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid number of DH parameters");
            // [TODO] THROW ERROR; INTERRUPT
        }
        int number_of_dh_rows = ((number_of_joints+1)/4);
        std::cout << "Number of rows:\n" << number_of_dh_rows << std::endl;
        dh_params.resize(number_of_dh_rows, 4);
        for (int i=0; i<number_of_dh_rows; i++){
            RCLCPP_INFO(get_logger(), "%d", i);
            dh_params(i, 0) = dh_vector[0 + 4*i];
            dh_params(i, 1) = dh_vector[1 + 4*i];
            dh_params(i, 2) = dh_vector[2 + 4*i];
            dh_params(i, 3) = dh_vector[3 + 4*i];
        }
        std::cout << "Here is the dh matrix:\n" << dh_params << std::endl;   
    }

private:
    // void timer_callback()
    // {
    //     auto message = std_msgs::msg::String();
    //     message.data = "Hello, world! " + std::to_string(count_++);
    //     // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //     publisher_->publish(message);
    // }
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // size_t count_;
    std::vector<double> dh_vector;
    int number_of_joints;
    MatrixXd dh_params;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}