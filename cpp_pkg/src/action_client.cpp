#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/navigate.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>

class ActionClient : public rclcpp::Node
{
public:
  using Navigate = custom_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  ActionClient() : Node("action_client")
  {
	  robot_position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
		  "robot_position", 10
	  );

	  timer_ = this->create_wall_timer(
		  std::chrono::milliseconds(300),
		  std::bind(&ActionClient::timer_callback, this)
	  );

	  action_client_ = rclcpp_action::create_client<Navigate>(this, "navigate");

	  RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
		
	  if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
	  {
		  RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
		  rclcpp::shutdown();
	  }
		  RCLCPP_INFO(this->get_logger(), "Action server is ready");
	}

	geometry_msgs::msg::Point desired;

	void send_goal(const geometry_msgs::msg::Point &goal_point)
	{
		auto goal_msg = Navigate::Goal();
		goal_msg.goal_point = goal_point;

		RCLCPP_INFO(this->get_logger(), "Sending goal: x = %f, y = %f", goal_point.x, goal_point.y);

		auto send_goal_options = rclcpp_action::Client<Navigate>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
		send_goal_options.feedback_callback =
			std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
		send_goal_options.result_callback =
			std::bind(&ActionClient::result_callback, this, std::placeholders::_1);

		action_client_->async_send_goal(goal_msg, send_goal_options);
	}

private:
	void goal_response_callback(GoalHandleNavigate::SharedPtr future)
	{
		auto goal_handle = future.get();
		if (!goal_handle)
		{
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
		}
	}

	void feedback_callback(
		GoalHandleNavigate::SharedPtr,
		const std::shared_ptr<const Navigate::Feedback> feedback)
	{
		dtp=feedback->distance_to_point;
		RCLCPP_INFO(this->get_logger(), "Feedback received: Distance to goal = %f", feedback->distance_to_point);
	}

	void result_callback(const GoalHandleNavigate::WrappedResult &result)
	{
		switch (result.code)
		{
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(this->get_logger(), "Goal succeeded! Elapsed time: %f seconds", result.result->elapsed_time);
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
			break;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
			break;
		default:
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			break;
		}
	}

	void timer_callback()
	{
		if (robot_position.x < desired.x)
		{
			robot_position.x++;

		}
		if (robot_position.y < desired.y)
		{
			robot_position.y++;
		
		}
		if (dtp!=0)
		{
			robot_position_publisher_->publish(robot_position);
			RCLCPP_INFO(this->get_logger(), "Current position: x: %f, y: %f", robot_position.x, robot_position.y);
		}
		else
			RCLCPP_INFO(this->get_logger(), "Goal Reached!");
	}

	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_position_publisher_;
	geometry_msgs::msg::Point robot_position;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp_action::Client<Navigate>::SharedPtr action_client_;
	std::atomic<float> dtp;

};


int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto action_client = std::make_shared<ActionClient>();
	auto &desired_point = action_client->desired;

	std::cout << "Enter desired x coordinate: ";
	std::cin >> desired_point.x;
	std::cout << "Enter desired y coordinate: ";
	std::cin >> desired_point.y;

	action_client->send_goal(desired_point);

	rclcpp::spin(action_client);
	rclcpp::shutdown();
	return 0;
}
