#include <memory>
#include <cmath>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tortoisebot_interfaces/action/waypoint.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class WaypointActionServer : public rclcpp::Node {
public:
    using Waypoint = tortoisebot_interfaces::action::Waypoint;
    using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

    explicit WaypointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("tortoisebot_as", options) {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Waypoint>(
            this,
            "tortoisebot_as",
            std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
            std::bind(&WaypointActionServer::handle_cancel, this, _1),
            std::bind(&WaypointActionServer::handle_accepted, this, _1)
        );

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server started");
    }

private:
    rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    geometry_msgs::msg::Point position_;
    double yaw_;

    geometry_msgs::msg::Point des_pos_;
    double yaw_precision_ = M_PI / 90; // +/- 2 degree allowed
    double dist_precision_ = 0.05;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        position_ = msg->pose.pose.position;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Waypoint::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with position (%f, %f, %f)",
                    goal->position.x, goal->position.y, goal->position.z);
        (void)uuid;
        des_pos_ = goal->position;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&WaypointActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(25);
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<Waypoint::Feedback>();
        auto result = std::make_shared<Waypoint::Result>();

        bool success = true;

        while (rclcpp::ok()) {
            double desired_yaw = std::atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
            double err_pos = std::sqrt(std::pow(des_pos_.y - position_.y, 2) + std::pow(des_pos_.x - position_.x, 2));
            double err_yaw = desired_yaw - yaw_;

            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            if (err_pos > dist_precision_) {
                if (std::fabs(err_yaw) > yaw_precision_) {
                    auto twist_msg = geometry_msgs::msg::Twist();
                    twist_msg.angular.z = (err_yaw > 0) ? 0.65 : -0.65;
                    cmd_vel_publisher_->publish(twist_msg);
                } else {
                    auto twist_msg = geometry_msgs::msg::Twist();
                    twist_msg.linear.x = 0.6;
                    twist_msg.angular.z = 0;
                    cmd_vel_publisher_->publish(twist_msg);
                }

                feedback->position = position_;
                feedback->state = "moving";
                goal_handle->publish_feedback(feedback);
            } else {
                break;
            }

            loop_rate.sleep();
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
        cmd_vel_publisher_->publish(twist_msg);

        if (success) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
