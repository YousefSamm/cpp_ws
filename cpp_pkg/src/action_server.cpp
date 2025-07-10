#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/action/navigate.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <functional>
#include <chrono>

const float DIST_THRESHOLD = 0.1;
class ActionServer : public rclcpp::Node
{
	public:
		ActionServer() : Node("action_server")
		{
			robot_position = geometry_msgs::msg::Point();
			robot_position_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
				"robot_position", 10,
				std::bind(&ActionServer::robot_position_callback, this, std::placeholders::_1)
			);
			action_server_ = rclcpp_action::create_server<custom_interfaces::action::Navigate>(
				this->get_node_base_interface(),
				this->get_node_clock_interface(),
				this->get_node_logging_interface(),
				this->get_node_waitables_interface(),
				"navigate",
				std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
				std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
				std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
			);
			RCLCPP_INFO(this->get_logger(), "Action server initiated");
		}	
	private:
		rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const custom_interfaces::action::Navigate::Goal> goal)
		{
			(void)	uuid;
			RCLCPP_INFO(this->get_logger(),
			 "Received goal request with x: %f, y: %f",
			 goal->goal_point.x, goal->goal_point.y);
			
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}
		rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_interfaces::action::Navigate>> goal_handle)
		{
			(void) goal_handle;
			RCLCPP_INFO(this->get_logger(), "Received cancel request");
			return rclcpp_action::CancelResponse::ACCEPT;
		}
		void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_interfaces::action::Navigate>> goal_handle)	
		{
			//start a new thread to avoid blocking ROS executer
			std::thread{
				std::bind(&ActionServer::execute, this,
				std::placeholders::_1),
				goal_handle}.detach();
		}
		void execute(
			const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_interfaces::action::Navigate>> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Executing goal");
			auto start_time = rclcpp::Clock().now();
			auto goal = goal_handle->get_goal();
			auto feedback = std::make_shared<custom_interfaces::action::Navigate::Feedback>();
			auto result = std::make_shared<custom_interfaces::action::Navigate::Result>();
			rclcpp::Rate loop_rate(1);
			feedback->distance_to_point = DIST_THRESHOLD;
			while (feedback->distance_to_point >= DIST_THRESHOLD)
			{
				feedback->distance_to_point = std::sqrt(
				std::pow(this->robot_position.x - goal->goal_point.x, 2) +
				std::pow(this->robot_position.y - goal->goal_point.y, 2)	
			);
			goal_handle->publish_feedback(feedback);
			loop_rate.sleep();
			}
			result->elapsed_time = (rclcpp::Clock().now() - start_time).seconds();
			goal_handle->succeed(result);
			finish_logger=true;
			RCLCPP_INFO(this->get_logger(), "Goal succeeded");
			RCLCPP_INFO(this->get_logger(), "Elapsed time: %f", result->elapsed_time);
		}
		void robot_position_callback(geometry_msgs::msg::Point::SharedPtr msg)
		{
			robot_position = *msg;
			if(!finish_logger){
			RCLCPP_INFO(this->get_logger(), "Robot position updated: x: %f, y: %f", robot_position.x, robot_position.y);
			}
		}
		
		rclcpp_action::Server<custom_interfaces::action::Navigate>::SharedPtr action_server_;
		geometry_msgs::msg::Point robot_position;
		rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position_subscription_;
		bool finish_logger= false;		
	};
int main(int argc, char* argv[])
{	
	rclcpp::init(argc,argv);
	auto node = std::make_shared<ActionServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
