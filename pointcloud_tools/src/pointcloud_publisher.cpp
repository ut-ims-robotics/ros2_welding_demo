#define BOOST_BIND_NO_PLACEHOLDERS 
#include <pcl/io/io.h>
#include <string> 
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/features/integral_image_normal.h>
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile ("/home/taaniel/poincloud_tools/test.pcd", *cloud);
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cloud, output);
      output.header.frame_id = "base_link";
      output.header.stamp = rclcpp::Clock().now();
      publisher_->publish(output);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
