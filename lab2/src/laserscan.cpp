#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class laserScan : public rclcpp::Node{
  public:
    laserScan() : Node("laserScan"){
          
      // inits publisher
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan",3);

      // init subscriber
      subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10, std::bind(&laserScan::callbackLaserScan,this,std::placeholders::_1));
    }

  private:
    // pub and sub definition (can be anywhere in the class)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      // init variables 
      sensor_msgs::msg::LaserScan newScan = *msg;
      std::vector<float> newRanges;

      // find the index for the angle min and max
      float angle_inc = msg->angle_increment;
      float NEW_ANGLE_MIN = M_PI/2;
      float NEW_ANGLE_MAX = M_PI;
      double index_min = NEW_ANGLE_MIN/angle_inc;
      double index_max = NEW_ANGLE_MAX/angle_inc;
      
      // need to specify angle min and max to make sure orientation is correct
      // without this it looks like auto starts from 0 and everything is on an offset
      newScan.angle_max = NEW_ANGLE_MAX;
      newScan.angle_min = NEW_ANGLE_MIN;
      
      for(int i = index_min; i < index_max; i++){ 
        newRanges.push_back(newScan.ranges.at(i));
      }
      newScan.ranges = newRanges;
      publisher_->publish(newScan);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laserScan>());
  rclcpp::shutdown();
  return 0;
}
