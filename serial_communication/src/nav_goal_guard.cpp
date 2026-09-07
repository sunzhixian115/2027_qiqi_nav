#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavGoalGuard : public rclcpp::Node
{
public:
  NavGoalGuard()
  : Node("nav_goal_guard")
  {
    this->declare_parameter<double>("goal_timeout", 1.0);
    this->get_parameter("goal_timeout", goal_timeout_);

    last_goal_time_ = this->now();

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&NavGoalGuard::goalCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&NavGoalGuard::checkTimeout, this));
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    last_goal_time_ = this->now();
  }

  void checkTimeout()
  {
    const auto now = this->now();
    const double dt = (now - last_goal_time_).seconds();
    if (dt <= goal_timeout_) {
      return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not available");
      return;
    }

    nav_client_->async_cancel_all_goals();
    RCLCPP_INFO(this->get_logger(), "Goal timeout (%.2fs). Cancel all nav goals.", dt);

    // Avoid canceling every cycle while no new goal arrives.
    last_goal_time_ = now;
  }

  double goal_timeout_{1.0};
  rclcpp::Time last_goal_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavGoalGuard>());
  rclcpp::shutdown();
  return 0;
}
