#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include <chrono>
const double RPM_DEFAULT_VALUE=100.0;
class RpmPublisher : public rclcpp::Node
{
	public: RpmPublisher() : Node("rpm_publisher")
	{
		this->declare_parameter<double>("rpm_value", RPM_DEFAULT_VALUE);
		publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpm", 10);
		timer_=this->create_wall_timer(std::chrono::seconds(1),
	std::bind(&RpmPublisher::timer_callback, this)
	);
	}
	private:
	void timer_callback()
	{
		auto rpm = std_msgs::msg::Float64();
		rclcpp::Parameter rpm_value_param_object= this->get_parameter("rpm_value");
		rpm.data=rpm_value_param_object.as_double();
		publisher_->publish(rpm);
		RCLCPP_INFO(this->get_logger(), "publishing %f", rpm.data);
		

	}
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	//double rpm_value=RPM_DEFAULT_VALUE;
};
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RpmPublisher>());
	rclcpp::shutdown();
	return 0;
}