#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/turn_camera.hpp"
#include <chrono>
#include <functional>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class TurnCameraServer : public rclcpp::Node
{
	public:
		TurnCameraServer(std::string exe_dir) : Node("turn_camera_server")
		{
			ws_dir_= get_ws_dir(exe_dir);
			service_=this->create_service<custom_interfaces::srv::TurnCamera>(
				"turn_camera",
				std::bind(&TurnCameraServer::turn_camera_callback, this, std::placeholders::_1, std::placeholders::_2)
			);
			
			
		}
		cv::Mat image;
		private: 
			void turn_camera_callback(custom_interfaces::srv::TurnCamera::Request::SharedPtr request,
			custom_interfaces::srv::TurnCamera::Response::SharedPtr response)
			{
				float closest_num = available_angles_[0];
				float angle_diff;
				float smallest_angle = request->degree_turn-available_angles_[0];
				RCLCPP_INFO(this->get_logger(), "Server Processing");
				for(int i=0; i<5; i++)
				{
					angle_diff = std::abs(request->degree_turn - available_angles_[i]);
					if (angle_diff < smallest_angle)
					{
						smallest_angle=angle_diff;
						closest_num=available_angles_[i];
					}
				}
					std::string image_path = ws_dir_ + "src/cpp_pkg/images/" + std::to_string((int)closest_num) + ".png";
					std::cout<< image_path << std::endl;
					auto image = cv::imread(image_path);
					auto img_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
					response->camera_image =*img_ptr;

			}
			std::string get_ws_dir(std::string exe_dir)
			{
				std::string::size_type substr_index = exe_dir.find("install");
				return exe_dir.substr(0,substr_index);
			}
			rclcpp::Service<custom_interfaces::srv::TurnCamera>::SharedPtr service_;
			std::ostringstream path_stream;
			int angle_int;
			std::string ws_dir_;
			const float available_angles_[5] {-30,-15,-0,15,30};
};
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurnCameraServer>(argv[0]));
	rclcpp::shutdown();
	return 0;
}