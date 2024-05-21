#include <cstdlib>
#include <memory>
#include <cmath>
#include <chrono>
#include <thread>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tortoisebot_interfaces/action/waypoint.hpp"

using namespace std::chrono_literals;

class WaypointActionClient : public rclcpp::Node {
public:
    using Waypoint = tortoisebot_interfaces::action::Waypoint;
    using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<Waypoint>;

    explicit WaypointActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("waypoint_action_client", options) {
        this->client_ptr_ = rclcpp_action::create_client<Waypoint>(this, "tortoisebot_as");
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WaypointActionClient::odom_callback, this, std::placeholders::_1)
        );
    }

    bool send_goal(const geometry_msgs::msg::Point & goal_position) {
        if (!this->client_ptr_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = Waypoint::Goal();
        goal_msg.position = goal_position;

        auto send_goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&WaypointActionClient::result_callback, this, std::placeholders::_1);

        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        auto status = goal_handle_future.wait_for(30s);
        if (status == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Send goal call timed out");
            return false;
        }
        goal_handle_ = goal_handle_future.get();
        if (!goal_handle_) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        return true;
    }

    nav_msgs::msg::Odometry::SharedPtr get_odom_data() const {
        return this->odom_data_;
    }

    bool wait_for_result() {
        if (!goal_handle_) {
            return false;
        }

        auto result_future = this->client_ptr_->async_get_result(goal_handle_);
        auto status = result_future.wait_for(30s);
        if (status == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Get result call timed out");
            return false;
        }

        auto wrapped_result = result_future.get();
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Goal did not succeed");
            return false;
        }

        return true;
    }

private:
    rclcpp_action::Client<Waypoint>::SharedPtr client_ptr_;
    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr goal_handle_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odometry data received");
        this->odom_data_ = msg;
    }

    void goal_response_callback(std::shared_ptr<GoalHandleWaypoint> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleWaypoint::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
        }
    }
};

class WaypointActionTest : public ::testing::Test {
protected:
    static nav_msgs::msg::Odometry::SharedPtr last_odom_data_;
    bool stop_spin_;

    void SetUp() override {
        action_client_node_ = std::make_shared<WaypointActionClient>();
        executor_.add_node(action_client_node_);
        stop_spin_ = false;
        spin_thread_ = std::make_unique<std::thread>([this]() {
            while (rclcpp::ok() && !stop_spin_) {
                executor_.spin_some();
            }
        });
    }

    void TearDown() override {
        stop_spin_ = true;
        if (spin_thread_ && spin_thread_->joinable()) {
            spin_thread_->join();
        }
        executor_.cancel();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<WaypointActionClient> action_client_node_;
    std::unique_ptr<std::thread> spin_thread_;
};

nav_msgs::msg::Odometry::SharedPtr WaypointActionTest::last_odom_data_ = nullptr;

// Global variables to hold input parameters
double goal_x = 0.0;
double goal_y = 0.0;

TEST_F(WaypointActionTest, TestEndPosition) {
    geometry_msgs::msg::Point goal_position;
    goal_position.x = goal_x;
    goal_position.y = goal_y;
    goal_position.z = 0.0;

    bool goal_sent = action_client_node_->send_goal(goal_position);
    ASSERT_TRUE(goal_sent) << "Failed to send goal";

    bool result_received = action_client_node_->wait_for_result();
    ASSERT_TRUE(result_received) << "Failed to receive result";

    last_odom_data_ = action_client_node_->get_odom_data();
    ASSERT_NE(last_odom_data_, nullptr) << "Odometry data not received";

    auto current_position = last_odom_data_->pose.pose.position;
    double error_margin = 0.05;  // Allowable error margin for position

    EXPECT_NEAR(current_position.x, goal_position.x, error_margin) << "Final X position is incorrect";
    EXPECT_NEAR(current_position.y, goal_position.y, error_margin) << "Final Y position is incorrect";
}

TEST_F(WaypointActionTest, TestEndYaw) {
    auto odom_data = last_odom_data_;
    ASSERT_NE(odom_data, nullptr) << "Odometry data not received";

    auto orientation = odom_data->pose.pose.orientation;
    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double goal_yaw = std::atan2(goal_y, goal_x);
    double error_margin = M_PI / 90;  // Allowable error margin for yaw

    EXPECT_NEAR(yaw, goal_yaw, 10 * error_margin) << "Final Yaw is incorrect";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv); // Initialize ROS context

    // Read input parameters from environment variables
    const char* env_x = std::getenv("GOAL_X");
    const char* env_y = std::getenv("GOAL_Y");

    if (env_x && env_y) {
        goal_x = std::stod(env_x);
        goal_y = std::stod(env_y);
    }

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown(); // Shutdown ROS context
    return result;
}