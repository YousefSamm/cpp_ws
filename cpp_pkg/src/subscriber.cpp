#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"

class HelloWorldSubNode : public rclcpp::Node
{
	public: HelloWorldSubNode() : Node("hello_world_subscriber")
	{
		subscriber_ = this->create_subscription<std_msgs::msg::String>(
			"hello_world",
			rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
			std::bind(&HelloWorldSubNode::sub_callback, this, std::placeholders::_1));
			cmd_subscriber_=this->create_subscription<geometry_msgs::msg::Twist>(
				"cmd_vel",
				rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
				std::bind(&HelloWorldSubNode::cmd_callback, this, std::placeholders::_1)
			);
	}
			private:
	void sub_callback(const std_msgs::msg::String::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(),"Received: '%s'", msg->data.c_str());
	}
	void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "linear velocity: '%f'", msg->linear.x);
	}
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; 
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
};
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HelloWorldSubNode>());
	rclcpp::shutdown();
	return 0;

}