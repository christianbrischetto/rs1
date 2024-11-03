#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class deadreckoning : public rclcpp::Node{
  public:
    deadreckoning() : Node("deadreckoning"){
      // inits publisher
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

      // init subscriber
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10, std::bind(&deadreckoning::odom_callback,this,std::placeholders::_1));

      noisy_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/noisy_odom",10);  

      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&deadreckoning::timer_callback, this));
    }

    void move(double linear_speed, double angular_speed, double distance, bool forward){
        geometry_msgs::msg::Twist msg;
        if (forward){msg.linear.x = linear_speed;}
        else{msg.linear.x = -linear_speed;} 

        msg.angular.z = angular_speed;

        // ******************************Calculate the time to move******************************
        double time_to_move = distance / linear_speed;
        rclcpp::Time start_time = this->now();

        rclcpp::Rate rate(10);
        while (rclcpp::ok())
        {
            // Calculate elapsed time
            rclcpp::Duration elapsed_time = this->now() - start_time;

            // If the elapsed time is greater than the time to move, stop the robot
            if (elapsed_time.seconds() >= time_to_move){
                break;
            }

            // Publish the command to move the robot
            cmd_vel_publisher_->publish(msg);
            rate.sleep();
        }

        // Stop the robot after moving
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(msg);
    }

  private:
    // pub and sub definition (can be anywhere in the class)

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      if(!initialised_){
        current_odom_ = *msg;
        initialised_ = true;
      }
    }

    void timer_callback(){
      //calc odom from current sensor feedback


      //add noise and publish
      odom_noisy_ = add_noise(current_odom_);
      noisy_odom_publisher_->publish(odom_noisy_);
    }

    nav_msgs::msg::Odometry add_noise(nav_msgs::msg::Odometry odom){
      // Add Gaussian noise to the odometry position and orientation
      odom.pose.pose.position.x += noise_dist_(gen_);
      odom.pose.pose.position.y += noise_dist_(gen_);
      odom.pose.pose.position.z += noise_dist_(gen_);
      odom.pose.pose.orientation.x += noise_dist_(gen_);
      odom.pose.pose.orientation.y += noise_dist_(gen_);
      odom.pose.pose.orientation.z += noise_dist_(gen_);
      odom.pose.pose.orientation.w += noise_dist_(gen_);

      // Add Gaussian noise to the odometry linear and angular velocity
      odom.twist.twist.linear.x += noise_dist_(gen_);
      odom.twist.twist.linear.y += noise_dist_(gen_);
      odom.twist.twist.linear.z += noise_dist_(gen_);
      odom.twist.twist.angular.x += noise_dist_(gen_);
      odom.twist.twist.angular.y += noise_dist_(gen_);
      odom.twist.twist.angular.z += noise_dist_(gen_);
      return(odom);
    }

    // init variables
    bool initialised_ = false;
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Odometry odom_noisy_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<deadreckoning>();
    // Example usage: Move forward with 0.2 m/s linear speed, 0.0 angular speed, for 1.0 meter
    node->move(0.2, 0.0, 10, true);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
