#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/odd_even_check.hpp"
#include <chrono>
#include <iostream>

class ClientNode : public rclcpp::Node
{
	public:
		ClientNode() : Node("odd_even_client_node")
		{
			client_ = this->create_client<custom_interfaces::srv::OddEvenCheck>("odd_even_check");
			while (!client_->wait_for_service(std::chrono::seconds(1)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for the service");
					return;
				}
				RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
			}
		}

		void send_request(int number)
		{
			auto request = std::make_shared<custom_interfaces::srv::OddEvenCheck::Request>();
			request->number = number;

			RCLCPP_INFO(this->get_logger(), "Sending request: number = %d", number);
			auto result_future = client_->async_send_request(request);

			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_INFO(this->get_logger(), "Result: %s", result_future.get()->decision.c_str());
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to call service");
			}
		}

	private:
		rclcpp::Client<custom_interfaces::srv::OddEvenCheck>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto client_node = std::make_shared<ClientNode>();

	int number;
	std::cout << "Enter a number: ";
	if (!(std::cin >> number)) {
		std::cerr << "Invalid input. Please enter an integer." << std::endl;
		return 1;
	}

	client_node->send_request(number);

	rclcpp::shutdown();
	return 0;
}
