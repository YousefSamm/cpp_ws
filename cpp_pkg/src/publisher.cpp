#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>



class HelloWorldPubNode : public rclcpp::Node
{
	public:
		HelloWorldPubNode() : Node("hello_world_pub_node")
		{
			cmd_publisher_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			publisher_=this->create_publisher<std_msgs::msg::String>("hello_world", 10);
			timer_=this->create_wall_timer(std::chrono::seconds(1),
			std::bind(&HelloWorldPubNode::publish_hello_world, this)
		 );
		}
		private:
		void publish_hello_world()
		{
			auto command = geometry_msgs::msg::Twist();
			auto message = std_msgs::msg::String();
			message.data="Hello World " + std::to_string(counter_);
			command.linear.x=1.0;
			cmd_publisher_->publish(command);
			publisher_->publish(message);
			counter_++;
		}
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		
		size_t counter_ = 0;
};
int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HelloWorldPubNode>());
	rclcpp::shutdown();

	return 0;
}