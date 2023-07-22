#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>
#include <memory>
#include <sstream>

using namespace std;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
turtlesim::msg::Pose::SharedPtr turtlesim_pose;

const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward, const rclcpp::Node::SharedPtr &node);

void rotate(double angular_speed, double relative_angle, bool clockwise, const rclcpp::Node::SharedPtr &node);

double degrees2radians(double angle_in_degrees);

void setDesiredOrientation(double desired_angle_radians, const rclcpp::Node::SharedPtr &node);

void poseCallback(const turtlesim::msg::Pose::SharedPtr pose_message);

void goToGoal(turtlesim::msg::Pose goal_pose, double distance_tolerance, const rclcpp::Node::SharedPtr &node);

void gridClean(const rclcpp::Node::SharedPtr &node);

void spiralClean(const rclcpp::Node::SharedPtr &node);


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("turtlesim_cleaner");
    turtlesim_pose = std::make_shared<turtlesim::msg::Pose>();

    velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1000);

    pose_subscriber = node->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, poseCallback);

    RCLCPP_INFO(node->get_logger(), "START MOVING");
    rclcpp::Rate loop_rate(0.5);

    gridClean(node);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

void move(double speed, double distance, bool isForward, const rclcpp::Node::SharedPtr &node) {
    auto vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Set a random linear velocity in the x-axis
    if (isForward)
        vel_msg->linear.x = abs(speed);
    else
        vel_msg->linear.x = -abs(speed);
    vel_msg->linear.y = 0;
    vel_msg->linear.z = 0;

    // Set a random angular velocity in the y-axis
    vel_msg->angular.x = 0;
    vel_msg->angular.y = 0;
    vel_msg->angular.z = 0;

    double t0 = rclcpp::Clock().now().seconds();
    double current_distance = 0.0;
    rclcpp::Rate loop_rate(100);

    do {
        velocity_publisher->publish(*vel_msg);
        double t1 = rclcpp::Clock().now().seconds();
        current_distance = speed * (t1 - t0);

        rclcpp::spin_some(node);
        loop_rate.sleep();
        cout << (t1 - t0) << ", " << current_distance << ", " << distance << endl;
    } while (current_distance < distance);

    vel_msg->linear.x = 0;
    velocity_publisher->publish(*vel_msg);
}

void rotate(double angular_speed, double relative_angle, bool clockwise, const rclcpp::Node::SharedPtr &node) {
    auto vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Set a random linear velocity in the x-axis
    vel_msg->linear.x = 0;
    vel_msg->linear.y = 0;
    vel_msg->linear.z = 0;

    // Set a random angular velocity in the y-axis
    vel_msg->angular.x = 0;
    vel_msg->angular.y = 0;

    if (clockwise)
        vel_msg->angular.z = -abs(angular_speed);
    else
        vel_msg->angular.z = abs(angular_speed);

    double current_angle = 0.0;
    double t0 = rclcpp::Clock().now().seconds();
    rclcpp::Rate loop_rate(10); // Publish the velocity at 10 Hz (10 times/ sec)

    do {
        velocity_publisher->publish(*vel_msg);
        double t1 = rclcpp::Clock().now().seconds();
        current_angle = angular_speed * (t1 - t0);

        rclcpp::spin_some(node);

        loop_rate.sleep();
    } while (current_angle < relative_angle);

    // Finally, stop the robot when the distance is moved
    vel_msg->angular.z = 0;
    velocity_publisher->publish(*vel_msg);
}

double degrees2radians(double angle_in_degrees) {
    return angle_in_degrees * PI / 180.0;
}

void setDesiredOrientation(double desired_angle_radians, const rclcpp::Node::SharedPtr &node) {
    double relative_angle_radians = desired_angle_radians - turtlesim_pose->theta;
    bool clockwise = ((relative_angle_radians < 0) ? true : false);

    rotate(degrees2radians(10), abs(relative_angle_radians), clockwise, node);
}

void poseCallback(const turtlesim::msg::Pose::SharedPtr pose_message) {
    turtlesim_pose->
            x = pose_message->x;
    turtlesim_pose->
            y = pose_message->y;
    turtlesim_pose->
            theta = pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

void rotateToGoal(turtlesim::msg::Pose goal_pose, const std::shared_ptr<geometry_msgs::msg::Twist> &vel_msg, const rclcpp::Node::SharedPtr &node) {
    double theta, Ki = 1.5;
    rclcpp::Rate loop_rate(10);
    do {
        // Angular velocity in the z-axis
        theta = Ki * (atan2(goal_pose.y - turtlesim_pose->y, goal_pose.x - turtlesim_pose->x) - turtlesim_pose->theta);
        cout << "theta: " << theta << endl;

        vel_msg->angular.x = 0;
        vel_msg->angular.y = 0;
        vel_msg->angular.z = theta;

        velocity_publisher->publish(*vel_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    } while (abs(theta) != 0);

    RCLCPP_INFO(node->get_logger(), "END ROTATE");
}

void moveToGoal(turtlesim::msg::Pose goal_pose, double distance_tolerance, const std::shared_ptr<geometry_msgs::msg::Twist> &vel_msg, const rclcpp::Node::SharedPtr &node) {
    double E, e, Kp = 0.8;
    rclcpp::Rate loop_rate(10);
    do {
        if (e <= distance_tolerance)
            rotateToGoal(goal_pose, vel_msg, node);

        e = getDistance(turtlesim_pose->x, turtlesim_pose->y, goal_pose.x, goal_pose.y);
        cout << "distance: " << e << endl;

        E = E + e;

        vel_msg->linear.x = (Kp * e);
        vel_msg->linear.y = 0;
        vel_msg->linear.z = 0;

        velocity_publisher->publish(*vel_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    } while (e > distance_tolerance);

    cout << "total: " << E << endl;
    RCLCPP_INFO(node->get_logger(), "END MOVE");
}

void goToGoal(turtlesim::msg::Pose goal_pose, double distance_tolerance, const rclcpp::Node::SharedPtr &node) {
    RCLCPP_INFO(node->get_logger(), "MOVING TO GOAL");

    auto vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

    moveToGoal(goal_pose, distance_tolerance, vel_msg,node);

    rclcpp::Rate loop_rate(10);
    vel_msg->linear.x = 0;
    vel_msg->angular.z = 0;
    velocity_publisher->publish(*vel_msg);
}

void gridClean(const rclcpp::Node::SharedPtr &node) {
    rclcpp::Rate loop(0.5);
    turtlesim::msg::Pose pose;
    pose.x = 1;
    pose.y = 1;
    pose.theta = 0;

    goToGoal(pose, 0.01, node);
    loop.sleep();

    double distance = getDistance(turtlesim_pose->x, turtlesim_pose->y, x_max, y_max);

    stringstream ss;
    ss << "DISTANCE: " << distance << endl;
    RCLCPP_INFO(node->get_logger(), ss.str().c_str());
}