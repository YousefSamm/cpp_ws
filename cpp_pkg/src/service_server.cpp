#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/odd_even_check.hpp"
#include <functional>
#include <fmt/core.h>
typedef custom_interfaces::srv::OddEvenCheck OddEvenCheck;
class ServerNode : public rclcpp::Node
{
	public:
		ServerNode() : Node("server_node")
		{
			server_ = this->create_service<custom_interfaces::srv::OddEvenCheck>(
				"odd_even_check",
				std::bind(&ServerNode::check_odd_even, this, std::placeholders::_1, std::placeholders::_2)
			);
		}
	private:
		void check_odd_even(OddEvenCheck::Request::SharedPtr request, OddEvenCheck::Response::SharedPtr response)
		{
			RCLCPP_INFO(this->get_logger(), "Request received");
			if (std::abs(request->number) % 2 == 0)
			{
				response->decision =fmt::format("The number {} is Even", request->number);
				RCLCPP_INFO(this->get_logger(), "The number is even");
			}
			else
			{
				response->decision = fmt::format("the number {} is Odd",request->number);
				RCLCPP_INFO(this->get_logger(), "The number is odd");
			}
		}
		rclcpp::Service<custom_interfaces::srv::OddEvenCheck>::SharedPtr server_;
};




int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ServerNode>());
	rclcpp::shutdown();

	return 0;
}