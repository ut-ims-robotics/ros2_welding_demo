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

//pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
int pcd_file_counter = 0;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 10, std::bind(&MinimalSubscriber::pointCloud2Callback, this, _1));
    }

  private:
    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard");
        pcl::fromROSMsg(*msg, *cloud);
        // estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 

        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);
        ne.compute(*normals);

        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); 

        pcl::io::savePCDFileASCII ("test_pcd_"+std::to_string(pcd_file_counter)+".pcd", *cloud_with_normals);

        pcd_file_counter = pcd_file_counter+1;
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  rclcpp::shutdown();
  return 0;
}