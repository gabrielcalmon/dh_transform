#include <memory>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pose_calculator.hpp"

class PosePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PosePublisher(const std::string & nodeName, bool intraProcessComms = false)
  : rclcpp_lifecycle::LifecycleNode(
    nodeName,
    rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms)
  )
  {
  }

  void
  publish_pose()
  {
    // auto msg = std::make_unique<std_msgs::msg::String>();
    auto pose = std::make_unique<geometry_msgs::msg::Pose>();
    pose->orientation.x = 1.15;

    // msg->data = "Lifecycle Hello World" + std::to_string(++count);

    if (!publisher->is_activated())
    {
      RCLCPP_INFO(
        get_logger(), "Lifecycle publisher currently inactive");
    }
    // else
    // {
    //   RCLCPP_INFO(
    //     get_logger(), "Lifecycle publisher active. Publishing [%f]", pose->orientation.x);
    // }

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

    // initialize class variables
    poseCalculatorObj = std::make_unique<PoseCalculator>("name", number_of_links, link_lengths);
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
    
    // std::vector<double> initial_angles = std::vector<double>(3, 0);
    // poseCalculatorObj->calculateDhMatrix(initial_angles);

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

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
    // RCLCPP_INFO(this->get_logger(), "I heard a joint_state msg");
    for(int joint=0; joint < number_of_links; joint++){
      joint_angles_values[joint]=joint_state->position[joint];
      // std::cout << joint_angles_values[joint] << " ";
    }
    // std::cout << std::endl;
  }

};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<PosePublisher> lc_publisher_node = std::make_shared<PosePublisher>("pose_publisher");

  executor.add_node(lc_publisher_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}