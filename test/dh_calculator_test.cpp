#include "../include/pose_calculator.hpp"
#include "gtest/gtest.h"

double dh_tolerance = 1E-5;     // Absolute error
double pose_tolerance = 1E-3;

TEST(PoseCalculatorTests, CalculateDHparametersThreeLinksZeroAngles){
    int number_of_links = 3;
    std::vector<double> joint_angles_values = std::vector<double>(number_of_links, 0);
    std::vector<double> link_lengths = {0.1, 0.2, 0.3};
    MatrixXd result_dh;
    result_dh.resize(number_of_links+1,4);

    PoseCalculator poseCalculatorObj("test_node", number_of_links, link_lengths);
    result_dh = poseCalculatorObj.calculateDhMatrix(joint_angles_values);

    MatrixXd expected_dh{
      {0, 0, 0, 0},
      {0.1, 0, 0, 0},
      {0.2, 0, 0, 0},
      {0.3, 0, 0, 0}
    };
    
    for(int row=0; row<=number_of_links; row++){
      for(int column=0; column<4; column++){
        EXPECT_NEAR(result_dh(row, column), expected_dh(row, column), dh_tolerance);
      }
    }
}

TEST(PoseCalculatorTests, CalculateDHparametersThreeLinksNonZeroAngles){
    int number_of_links = 3;
    std::vector<double> joint_angles_values = {30, 45, 60};
    std::vector<double> link_lengths = {0.115, 0.293, 0.365};
    MatrixXd result_dh;
    result_dh.resize(number_of_links+1,4);

    PoseCalculator poseCalculatorObj("test_node", number_of_links, link_lengths);
    result_dh = poseCalculatorObj.calculateDhMatrix(joint_angles_values);

    MatrixXd expected_dh{
      {0, 0, 0, 30},
      {0.115, 0, 0, 45},
      {0.293, 0, 0, 60},
      {0.365, 0, 0, 0}
    };
    
    for(int row=0; row<=number_of_links; row++){
      for(int column=0; column<4; column++){
        EXPECT_NEAR(result_dh(row, column), expected_dh(row, column), dh_tolerance);
      }
    }  
}

TEST(PoseCalculatorTests, CalculateDHparametersSixLinksNonZeroAngles){
    int number_of_links = 6;
    std::vector<double> joint_angles_values = {30, 45, 60, 45, 30, 60};
    std::vector<double> link_lengths = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    MatrixXd result_dh;
    result_dh.resize(number_of_links+1,4);

    PoseCalculator poseCalculatorObj("test_node", number_of_links, link_lengths);
    result_dh = poseCalculatorObj.calculateDhMatrix(joint_angles_values);

    MatrixXd expected_dh{
      {0, 0, 0, 30},
      {0.1, 0, 0, 45},
      {0.2, 0, 0, 60},
      {0.3, 0, 0, 45},
      {0.4, 0, 0, 30},
      {0.5, 0, 0, 60},
      {0.6, 0, 0, 0}
    };
    
    for(int row=0; row<=number_of_links; row++){
      for(int column=0; column<4; column++){
        EXPECT_NEAR(result_dh(row, column), expected_dh(row, column), dh_tolerance);
      }
    }
}

TEST(PoseCalculatorTests, CalculateEndEffectorPoseThreeLinks){
    int number_of_links = 3;
    std::vector<double> joint_angles_values = {5, 15, 90};;
    std::vector<double> link_lengths = {0.1, 0.2, 0.3};
    MatrixXd result_dh;
    result_dh.resize(number_of_links+1,4);

    PoseCalculator poseCalculatorObj("test_node", number_of_links, link_lengths);
    poseCalculatorObj.calculateDhMatrix(joint_angles_values);

    auto result_pose = geometry_msgs::msg::Pose();
    result_pose = poseCalculatorObj.dh2endEffectorPose();

    auto expected_pose = geometry_msgs::msg::Pose();
    expected_pose.position.x = 0.1850;
    expected_pose.position.y = 0.3590;
    expected_pose.position.z = 0.0;

    expected_pose.orientation.w = 0.5736;
    expected_pose.orientation.x = 0.0;
    expected_pose.orientation.y = 0.0;
    expected_pose.orientation.z = 0.8191;
    
    EXPECT_NEAR(result_pose.position.x, expected_pose.position.x, pose_tolerance);
    EXPECT_NEAR(result_pose.position.y, expected_pose.position.y, pose_tolerance);
    EXPECT_NEAR(result_pose.position.z, expected_pose.position.z, pose_tolerance);

    EXPECT_NEAR(result_pose.orientation.x, expected_pose.orientation.x, pose_tolerance);
    EXPECT_NEAR(result_pose.orientation.y, expected_pose.orientation.y, pose_tolerance);
    EXPECT_NEAR(result_pose.orientation.z, expected_pose.orientation.z, pose_tolerance);
    EXPECT_NEAR(result_pose.orientation.w, expected_pose.orientation.w, pose_tolerance);
}

TEST(PoseCalculatorTests, CalculateEndEffectorPoseSixLinks){
    int number_of_links = 6;
    std::vector<double> joint_angles_values = {30, 45, 60, 45, 30, 60};
    std::vector<double> link_lengths = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    MatrixXd result_dh;
    result_dh.resize(number_of_links+1,4);

    PoseCalculator poseCalculatorObj("test_node", number_of_links, link_lengths);
    poseCalculatorObj.calculateDhMatrix(joint_angles_values);

    auto result_pose = geometry_msgs::msg::Pose();
    result_pose = poseCalculatorObj.dh2endEffectorPose();

    auto expected_pose = geometry_msgs::msg::Pose();
    expected_pose.position.x = -0.9068;
    expected_pose.position.y = -0.3947;
    expected_pose.position.z = 0.0;

    expected_pose.orientation.w = 0.7071;
    expected_pose.orientation.x = 0.0;
    expected_pose.orientation.y = 0.0;
    expected_pose.orientation.z = -0.7071;
    
    EXPECT_NEAR(result_pose.position.x, expected_pose.position.x, pose_tolerance);
    EXPECT_NEAR(result_pose.position.y, expected_pose.position.y, pose_tolerance);
    EXPECT_NEAR(result_pose.position.z, expected_pose.position.z, pose_tolerance);

    EXPECT_NEAR(result_pose.orientation.x, expected_pose.orientation.x, pose_tolerance);
    EXPECT_NEAR(result_pose.orientation.y, expected_pose.orientation.y, pose_tolerance);
    EXPECT_NEAR(result_pose.orientation.z, expected_pose.orientation.z, pose_tolerance);
    EXPECT_NEAR(result_pose.orientation.w, expected_pose.orientation.w, pose_tolerance);
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  
  return result;
}