#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class sprint1 : public rclcpp::Node{
  public:
    sprint1() : Node("sprint_one"){
          
      // inits publisher
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan",3);

      // init subscriber
      subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10, std::bind(&sprint1::callbackLaserScan,this,std::placeholders::_1));
    }

  private:
    // pub and sub definition (can be anywhere in the class)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      // init variables 
      sensor_msgs::msg::LaserScan newScan = *msg;
      std::vector<float> newRanges;
      int n_var = 5;
      newScan.angle_increment = newScan.angle_increment*n_var;

      for(long unsigned int i = 0; i < newScan.ranges.size(); i++){ 
        if(i % n_var == 0){
            newRanges.push_back(newScan.ranges.at(i));
        }
      }
      newScan.ranges = newRanges;
      publisher_->publish(newScan);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sprint1>());
  rclcpp::shutdown();
  return 0;
}
