#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include <chrono>
const double wheel_radius_default = 0.2;
class SpeedCalculator : public rclcpp::Node
{
	public:
    SpeedCalculator() : Node("speed_calculator")
	{
		this->declare_parameter<double>("wheel_radius", wheel_radius_default);
		subscriber_=this->create_subscription<std_msgs::msg::Float64>("rpm", 
			rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
			std::bind(&SpeedCalculator::subs_callback, this, std::placeholders::_1)
	);
		publisher_=this->create_publisher<std_msgs::msg::Float64>("speed",10);
		timer_=this->create_wall_timer(std::chrono::seconds(1),
		std::bind(&SpeedCalculator::timer_callback, this)
	);
	}
	private:
	void subs_callback(std_msgs::msg::Float64::SharedPtr msg)
	{
		rpm = msg->data;
	}
	void timer_callback()
	{	
		rclcpp::Parameter wheel_radius_param = this->get_parameter("wheel_radius");
		auto speed = std_msgs::msg::Float64();
		speed.data = rpm*(2*pi/60.0)*wheel_radius_param.as_double();
		RCLCPP_INFO(this->get_logger(), "publishing speed: %f", speed.data);
		publisher_->publish(speed);
	}
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	double rpm;

	double pi = 3.14;
};
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpeedCalculator>());
	rclcpp::shutdown();
	return 0;
}