#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Eigen::MatrixXd, Eigen::Matrix4d, Eigen::Vector4d, Eigen::VectorXd;

class DhCalculator : public rclcpp::Node
{
public:
    DhCalculator()
    : Node("minimal_publisher")
    {        
        this->declare_parameter("number_of_links", 0);
        number_of_links = get_parameter("number_of_links").as_int();

        // [TODO] TRATAR CASO DE INPUT INT
        this->declare_parameter("links_lenght", std::vector<double>(number_of_links,-1));
        dh_vector = get_parameter("links_lenght").as_double_array();

        // std::cout << dh_vector[number_of_links-1] << std::endl;

        // THIS CODE WAS NOT SUPOSED TO BE ON CONSTRUCTOR
        if(number_of_links != int(dh_vector.size())){    // It is expected that the DH params come in multiples of 4
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The number of links passed doesn't match the number of lenghts");
            // [TODO] THROW ERROR; INTERRUPT CODE
        }    
        std::vector<double> initial_angles = std::vector<double>(number_of_links, 0);
        dh_matrix = calculateDhMatrix(dh_vector, initial_angles);

        int number_of_dh_rows = static_cast<int>(dh_matrix.rows());
        
        end_effector_pose = calculateEndEffectorPose(dh_matrix, number_of_dh_rows);

        Eigen::Quaterniond end_effector_quaternion(end_effector_pose.block<3,3>(0,0));

        std::cout << "End effector quarternion: " << end_effector_quaternion << std::endl;        
    }

private:
    int number_of_links;
    std::vector<double> dh_vector;

    Matrix4d end_effector_pose;
    MatrixXd dh_matrix;

    // Methods
    double deg2rad(double deg);
    MatrixXd calculateDhMatrix(std::vector<double> link_lenghts, std::vector<double> angles);
    Matrix4d dh2Transform(Vector4d dh_line);
    Matrix4d calculateEndEffectorPose(MatrixXd dh_matrix, int num_of_lines);
};

double DhCalculator::deg2rad(double deg){
    return deg*M_PI/180;
}

MatrixXd DhCalculator::calculateDhMatrix(std::vector<double> link_lenghts, std::vector<double> angles){
    int num_links = link_lenghts.size();
    int num_joints = angles.size();

    if(num_links != num_joints){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The number of links should be equal to the number of joints");
    }

    MatrixXd dh_matrix(num_joints+1, 4);    // a dh matrix has 4 columns and (num_joints+1) rows
    dh_matrix << MatrixXd::Zero(4,4);
    
    for(int i=0; i<=num_joints; i++){
        if(i == 0){     // frame 0 (base)
            dh_matrix(i,3) = angles[i];
        } else if (i == num_joints){    // end effector
            dh_matrix(i,0) = link_lenghts[i];
        } else {
            dh_matrix(i,0) = link_lenghts[i];
            dh_matrix(i,3) = angles[i];
        }
    }

    std::cout << "output dh matrix:\n" << dh_matrix <<std::endl;
    return dh_matrix;
}

Matrix4d DhCalculator::dh2Transform(Vector4d dh_line){
    double alfaRad = deg2rad(dh_line(0));
    double a = dh_line(1);  // alpha
    double d = dh_line(2);
    double thetaRad = deg2rad(dh_line(3));
    Matrix4d transform{
        {cos(thetaRad),             -1*sin(thetaRad),             0,                a},
        {sin(thetaRad)*cos(alfaRad), cos(thetaRad)*cos(alfaRad), -1*sin(alfaRad), -1*sin(alfaRad)*d},
        {sin(thetaRad)*sin(alfaRad), cos(thetaRad)*sin(alfaRad), cos(alfaRad), cos(alfaRad)*d},
        {0,                          0,                          0,                 1}
    };
    
    return transform;
}

Matrix4d DhCalculator::calculateEndEffectorPose(MatrixXd dh_matrix, int num_of_lines){
    Matrix4d end_pose;
    end_pose << MatrixXd::Identity(4,4);

    for(int i=0; i<num_of_lines; i++){
        Vector4d dh_line = dh_matrix.row(i);
        end_pose *= dh2Transform(dh_line);
    }

    std::cout << "out put transform:\n" << end_pose <<std::endl;

    return end_pose;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DhCalculator>());
  rclcpp::shutdown();
  return 0;
}