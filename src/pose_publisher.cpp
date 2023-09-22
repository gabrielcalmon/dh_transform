#include <memory>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pose_calculator.hpp"
#include "../include/pose_publisher_exceptions.hpp"

// This classes uses the PoseCalculator class to get the end effector pose and is managed by the life cycle
class PosePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PosePublisher(const std::string & nodeName, bool intraProcessComms = false)
  : rclcpp_lifecycle::LifecycleNode(
    nodeName,
    rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms)
  )
  {
    RCLCPP_INFO(get_logger(), "Then node '%s' from PosePublisher class was successfully initialized", nodeName.c_str());
  }

  void publish_pose()
  {
    // Recalculate the DH matrix based on the new joint angles
    poseCalculatorObj->calculateDhMatrix(joint_angles_values);

    auto pose = geometry_msgs::msg::Pose();
    pose = poseCalculatorObj->dh2endEffectorPose();

    if (!publisher->is_activated())
    {
      RCLCPP_INFO(get_logger(), "Lifecycle publisher currently inactive");
    }
    else
    {
      poseCalculatorObj->printEndEffectorPose();
    }

    publisher->publish(std::move(pose));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    // declare and read parameters
    this->declare_parameter("publication_frequency_hz", 1.0);
    this->declare_parameter("number_of_links", 0);
    this->declare_parameter("links_lenght", std::vector<double>(number_of_links,-1));

    double publication_frequency_hz = get_parameter("publication_frequency_hz").as_double();
    number_of_links = get_parameter("number_of_links").as_int();

    // [TODO] TRATAR CASO DE INPUT INT
    std::vector<double> link_lengths = get_parameter("links_lenght").as_double_array();

    // Check for invalid data types in link_lengths (int or float)
    for (double length : link_lengths) {
        if (!std::is_same<double, decltype(length)>::value && !std::is_floating_point<decltype(length)>::value) {
            throw InvalidDataTypeException("Invalid data type detected in link_lengths");
        }
    }

    // initialize class variables
    poseCalculatorObj = std::make_unique<PoseCalculator>("pose_calculator", number_of_links, link_lengths);
    joint_angles_values = std::vector<double>(number_of_links, 0);

    // configure publish and subscription
    int64_t publication_rate_ms = static_cast<int64_t>(1000.0/publication_frequency_hz);

    publisher = this->create_publisher<geometry_msgs::msg::Pose>("/end_effector_pose",10);
    timer = this->create_wall_timer(
      std::chrono::milliseconds(publication_rate_ms), std::bind(&PosePublisher::publish_pose, this));

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_angles", 10, std::bind(&PosePublisher::joint_angles_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),"on_configure() called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  { 
    publisher->on_activate();
    RCLCPP_INFO(get_logger(),"on_activate() called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    publisher->on_deactivate();
    RCLCPP_INFO(get_logger(),"on_deactivate() called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    publisher.reset();
    timer.reset();
    RCLCPP_INFO(get_logger(),"on_cleanup() called");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(),"on_shutdown() called");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>> publisher;
  std::shared_ptr<rclcpp::TimerBase> timer;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::vector<double> joint_angles_values;
  int number_of_links;

  std::unique_ptr<PoseCalculator> poseCalculatorObj;

  void joint_angles_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
  {
    for(int joint=0; joint < number_of_links; joint++){
      joint_angles_values[joint]=joint_state->position[joint];
    }
  }
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  try{
    std::shared_ptr<PosePublisher> lc_publisher_node = std::make_shared<PosePublisher>("pose_publisher");

    executor.add_node(lc_publisher_node->get_node_base_interface());
    
    executor.spin();

    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
    rclcpp::shutdown();
  }

  return 0;
}