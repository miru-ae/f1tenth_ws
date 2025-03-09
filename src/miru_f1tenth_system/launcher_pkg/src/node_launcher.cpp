#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class NodeLauncher : public rclcpp::Node {
public:
    NodeLauncher() : Node("node_launcher") 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, 
            std::bind(&NodeLauncher::lidar_callback, this, std::placeholders::_1));
    }

private:
    std::string lidarscan_topic = "/scan";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Received Lidar scan with %zu ranges", scan_msg->ranges.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<NodeLauncher>();
    RCLCPP_INFO(node->get_logger(), "Launching NodeLauncher...");

    rclcpp::spin(node);
    rclcpp::shutdown();

  
  // 시스템 명령어 실행
  /*int result = std::system("ros2 run gap_follow reactive_node");*/
  /**/
  /*if (result == 0) {*/
  /*  printf("Successfully launched disparity_node\n");*/
  /*} else {*/
  /*  printf("Failed to launch disparity_node. Error code: %d\n", result);*/
  /*}*/
  
  return 0;
}
