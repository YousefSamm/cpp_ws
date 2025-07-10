#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
class TF_Listener : public rclcpp::Node
{
  public: 
    TF_Listener() : Node("tf_listener")
    {
      tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer, this);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&TF_Listener::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Node_started");
    }
    private:
    void timer_callback()
    {
      try 
      {
        geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform("map", "robot", tf2::TimePointZero);
        geometry_msgs::msg::Quaternion q_msg = transform.transform.rotation;
        tf2::Quaternion q_tf(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        tf2::Matrix3x3 m(q_tf);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Yaw %f", yaw);
        static int num_of_revolutions = 0;
        static float prev_yaw = 0.0f;
        if(yaw-prev_yaw < 0)
        {
          num_of_revolutions++;
          RCLCPP_INFO(this->get_logger(), "Number of revolutions: %d", num_of_revolutions);
        }
        prev_yaw = yaw;
      }
      catch (const tf2::TransformException & ex)
      {
        RCLCPP_INFO(this->get_logger(), "Could not Transform %s", ex.what());
      }
    }
      rclcpp::TimerBase::SharedPtr timer_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF_Listener>());
  rclcpp::shutdown();
  return 0;

}