#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

using Eigen::MatrixXd, Eigen::Matrix4d, Eigen::Vector4d, Eigen::VectorXd;

class PoseCalculator : public rclcpp::Node
{
public:
    PoseCalculator(const std::string & nodeName, int num_links, std::vector<double> l_lengths)
    : Node(nodeName), number_of_links(num_links), link_lengths(l_lengths)
    {
        // The number of links informed need to match the actual number of link's lengths
        if(number_of_links != int(link_lengths.size())){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "The number of links passed doesn't match the number of lengths");
            // [TODO] THROW ERROR; INTERRUPT CODE
        }

        // initialize DH matrix with zeros
        this->dh_matrix.resize(number_of_links+1,4);
        this->dh_matrix << MatrixXd::Zero(number_of_links+1,4);

        //initial joint angles are equal to zero
        std::vector<double> initial_angles = std::vector<double>(number_of_links, 0);
        this->dh_matrix = calculateDhMatrix(initial_angles);
    }

    MatrixXd calculateDhMatrix(std::vector<double> angles);
    geometry_msgs::msg::Pose dh2endEffectorPose();
    void printEndEffectorPose(bool quaternion_angles);

private:
    // properties
    int number_of_links;
    std::vector<double> link_lengths;

    geometry_msgs::msg::Pose end_effector_pose;
    MatrixXd dh_matrix;

    // Methods
    double deg2rad(double deg);
    double rad2deg(double rad);
    Matrix4d dh2Transform(Vector4d dh_line);
    Matrix4d calculateEndEffectorTransform(int num_of_lines);
    geometry_msgs::msg::Pose endEffectorTransformToPose(Matrix4d end_effector_transform);
    
};

double PoseCalculator::deg2rad(double deg){
    return deg*M_PI/180;
}

double PoseCalculator::rad2deg(double rad){
    return rad*180/M_PI;
}

MatrixXd PoseCalculator::calculateDhMatrix(std::vector<double> angles){
    int num_links = this->link_lengths.size();
    int num_joints = angles.size();

    if(num_links != num_joints){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The number of links should be equal to the number of joints");
    }    
    
    for(int nJoint=0; nJoint<=num_joints; nJoint++){
        if(nJoint == 0){                     // frame 0 (base)
            this->dh_matrix(nJoint,3) = angles[nJoint];
        } else if (nJoint == num_joints){    // end effector
            this->dh_matrix(nJoint,0) = this->link_lengths[nJoint-1];
        } else {
            this->dh_matrix(nJoint,0) = this->link_lengths[nJoint-1];
            this->dh_matrix(nJoint,3) = angles[nJoint];
        }
    }

    return this->dh_matrix;
}

Matrix4d PoseCalculator::dh2Transform(Vector4d dh_line){
    double a = dh_line(0);
    double alfaRad = deg2rad(dh_line(1));
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

Matrix4d PoseCalculator::calculateEndEffectorTransform(int num_of_lines){
    Matrix4d end_pose;
    end_pose << MatrixXd::Identity(4,4);

    for(int index=0; index<num_of_lines; index++){
        Vector4d dh_line = this->dh_matrix.row(index);
        end_pose *= dh2Transform(dh_line);
    }

    return end_pose;
}

geometry_msgs::msg::Pose PoseCalculator::endEffectorTransformToPose(Matrix4d end_effector_transform){
    Eigen::Quaterniond end_effector_quaternion(end_effector_transform.block<3,3>(0,0));
    this->end_effector_pose.orientation.x = end_effector_quaternion.x();
    this->end_effector_pose.orientation.y = end_effector_quaternion.y();
    this->end_effector_pose.orientation.z = end_effector_quaternion.z();
    this->end_effector_pose.orientation.w = end_effector_quaternion.w();

    this->end_effector_pose.position.x = end_effector_transform(0,3);
    this->end_effector_pose.position.y = end_effector_transform(1,3);
    this->end_effector_pose.position.z = end_effector_transform(2,3);

    return this->end_effector_pose;
}

geometry_msgs::msg::Pose PoseCalculator::dh2endEffectorPose(){
    int number_of_dh_rows = static_cast<int>(this->dh_matrix.rows());

    Matrix4d eff_pose = calculateEndEffectorTransform(number_of_dh_rows);

    return endEffectorTransformToPose(eff_pose);
}

void PoseCalculator::printEndEffectorPose(bool quaternion_angles=false){
    std::cout << std::endl <<"Translation: ";
    std::cout << this->end_effector_pose.position.x << " x +";
    std::cout << this->end_effector_pose.position.y << " y +";
    std::cout << this->end_effector_pose.position.z << " z" << std::endl;

    if(quaternion_angles){
        std::cout << "Orientation (Quaternion): ";
        std::cout << this->end_effector_pose.orientation.x << " x +";
        std::cout << this->end_effector_pose.orientation.y << " y +";
        std::cout << this->end_effector_pose.orientation.z << " z +";
        std::cout << this->end_effector_pose.orientation.w << " w" << std::endl;
    } else {
        tf2::Quaternion q(
        this->end_effector_pose.orientation.x,
        this->end_effector_pose.orientation.y,
        this->end_effector_pose.orientation.z,
        this->end_effector_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        std::cout << "Orientation (RPY - degrees): ";
        std::cout << rad2deg(roll) << " x +";
        std::cout << rad2deg(pitch) << " y +";
        std::cout << rad2deg(yaw) << " z" << std::endl;
    }
}