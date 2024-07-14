#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

class FuseScans : public rclcpp::Node
{
  public:
    FuseScans() : Node("fuse_scans"){
      lidar_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar_scan", 10, std::bind(&FuseScans::lidar_callback, this, _1));

      rclcpp::QoS qos_profile(10);
      qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

      lane_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/lanes_scan", qos_profile, std::bind(&FuseScans::lane_callback, this, std::placeholders::_1));

    }

  private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "For LIDAR I heard: '%s'", msg->header.frame_id.c_str());
    }

    void lane_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "For LANES I heard: '%s'", msg->header.frame_id.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lane_scan_sub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FuseScans>());
  rclcpp::shutdown();
  return 0;
}