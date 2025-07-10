#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/turn_camera.hpp"
#include "cv_bridge/cv_bridge.h"
#include <functional>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
class TurnCameraClient : public rclcpp::Node
{
public: 
	TurnCameraClient() : Node("turn_camera_client")
	{
		client_=this->create_client<custom_interfaces::srv::TurnCamera>("turn_camera");
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
	
	void send_request(float angle)
	{
		auto request = std::make_shared<custom_interfaces::srv::TurnCamera::Request>();
		request->degree_turn = angle;
		RCLCPP_INFO(this->get_logger(), "Sending request: angle = %f", angle);
		auto result_future = client_->async_send_request(request);

		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
			rclcpp::FutureReturnCode::SUCCESS)
		{
			auto response = result_future.get();
			cv::Mat cv_image = cv_bridge::toCvCopy(
					response->camera_image, sensor_msgs::image_encodings::BGR8)->image;
			cv::imshow("Camera's View", cv_image);
			cv::waitKey(0);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to call service");
		}
	

	}
	private:
	
	rclcpp::Client<custom_interfaces::srv::TurnCamera>::SharedPtr client_;

};
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto client_node = std::make_shared<TurnCameraClient>();
	float angle;
	std::cout<<"enter the camera angle: ";
	std::cin>>angle;
	client_node->send_request(angle);
	return 0;
}