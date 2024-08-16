#include <memory>
#include <queue>
#include <cassert>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

#define MAX_BUFFER_SIZE 50

class FuseScans : public rclcpp::Node
{
  public:
    FuseScans() : Node("fuse_scans"){
      lidar_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar_scan", 10, std::bind(&FuseScans::lidar_callback, this, _1));

      rclcpp::QoS qos_profile(10);
      qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

      lane_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lanes_scan", qos_profile, std::bind(&FuseScans::lane_callback, this, std::placeholders::_1)
      );

      fused_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/fused_scan", 10);
    }

  private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // RCLCPP_INFO(this->get_logger(), "For LIDAR I heard: '%s'", msg->header.frame_id.c_str());

      if (lane_buffer.size() > 0) {
        auto lane_msg = lane_buffer.front();
        lane_buffer.pop();
        // RCLCPP_INFO(this->get_logger(), "Consumed a lane msg. Remaining lane msgs: '%lu'", lane_buffer.size());
        // RCLCPP_INFO(this->get_logger(), "\nLIDAR SAMPLES: \n\n\t %lu \nLANE SAMPLES: \n\t %lu\n\n", msg->ranges.size(), lane_msg->ranges.size());

        // Perform fusion on the lidar messgae
        this->perform_fusion(msg->ranges, lane_msg->ranges, msg->ranges);
        auto fused_msg = *msg;  
        fused_scan_pub_->publish(fused_msg);
      } else {
        // RCLCPP_INFO(this->get_logger(), "Can't Consumed a lane msg because empty");
        fused_scan_pub_->publish(*msg);
      }

    }

    void lane_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // RCLCPP_INFO(this->get_logger(), "For LANES I heard: '%s'", msg->header.frame_id.c_str());

      if (this->lane_buffer.size() < MAX_BUFFER_SIZE) {
        this->lane_buffer.push(msg);
      } else {
        // RCLCPP_INFO(this->get_logger(), "Dropping lane scan message because message buffer full.");
      }
    }

    void perform_fusion(
      std::vector<float> &ranges_lidar,
      std::vector<float> &ranges_lane,
      std::vector<float> &ranges_fused) {

        assert(ranges_lidar.size() == ranges_lane.size()); 

        for (uint32_t i = 0; i < ranges_lidar.size(); i++) {
          ranges_fused[i] = std::min<float>(ranges_lidar[i], ranges_lane[i]);
        }
      }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lane_scan_sub_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_scan_pub_;

    std::queue<sensor_msgs::msg::LaserScan::SharedPtr> lane_buffer;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FuseScans>());
  rclcpp::shutdown();
  return 0;
}