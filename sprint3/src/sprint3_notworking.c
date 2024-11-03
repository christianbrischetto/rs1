#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <tuple>
#include <thread>
#include <utility>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class sprint3 : public rclcpp::Node
{
public:
    sprint3() : Node("map_processor_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&sprint3::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&sprint3::scanCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&sprint3::odom_callback, this, std::placeholders::_1));

        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        laserScan_ = *msg;
        if(mapped){
            laserScanToCvMat();
        }
    }

    // converts the map into an image and is saved as the global variable map_image_
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        map_image_ = occupancyGridToImage(mapMsg);
        mapped = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position data
        odom_ = *msg;
    }

    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        cv::Mat m_temp_img;
        cv::Mat m_MapBinImage;
        cv::Mat m_MapColImage;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x_ = grid->info.origin.position.x;
        origin_y_ = grid->info.origin.position.y;
        size_x_ = grid->info.width;
        size_y_ = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) <<  0, 0, 0,
                                                    0, 1, 0,
                                                    0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);
        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);
        cv::rotate(m_MapColImage, m_MapColImage, cv::ROTATE_90_COUNTERCLOCKWISE);
        std::cout << "Occupancy grid map converted to a binary image\n";

        return(m_MapColImage);
    }

    void laserScanToCvMat()
    {
        // Calculate center of the image (robot's position)
        int center_x = map_image_.rows / 2;
        int center_y = map_image_.cols / 2;
        float map_resolution = 0.05;
        cv::Mat temp_image;
        temp_image = map_image_.clone();
        std::vector<geometry_msgs::msg::Point> circle = getCircle(odom_);
        
        for(auto circ : circle){
            std::cout << "Circle Position: (" << circ.x << ", " << circ.y << ")" << std::endl;
            float x = circ.x + 1;
            float y = circ.y - 5;
            
            // Convert world coordinates (x, y) to pixel coordinates
            int pixel_x = static_cast<int>(center_x + x / map_resolution);
            int pixel_y = static_cast<int>(center_y - y / map_resolution); // Invert y-axis for image coordinates
            
            circle_loc_.x = pixel_x; 
            circle_loc_.y = pixel_y;
            
            // Draw a circle around the found location in the MAP image
            
            cv::circle(temp_image, circle_loc_, 3, cv::Scalar(255, 0, 255), 2);
        }

        cv::imshow(WINDOW1, temp_image);
        cv::waitKey(1);
    }

    // convert polar coordinates (distance and angle) to cartesian coords in the frame of the robot
    geometry_msgs::msg::Point polarToCart(unsigned int index)
    {
        float angle = laserScan_.angle_min + laserScan_.angle_increment*index + 1.5708;
        float range = laserScan_.ranges.at(index);
        geometry_msgs::msg::Point cart;
        cart.x = static_cast<double>(range*cos(angle));
        cart.y = static_cast<double>(range*sin(angle));
        return cart;
    }

    std::vector<std::vector<int>> getSegments()
    {
        std::vector<std::vector<int>> segments; // vector of clusters
        std::vector<int> cluster;               // vector of indexes for laserScan range values within a single cluster/group
        std::vector<float> ranges = laserScan_.ranges;

        sensor_msgs::msg::LaserScan laserScan = laserScan_;   

        // obtains only valid readings
        std::vector<double> objectIndex;
        for (unsigned int i = 0; i < laserScan.ranges.size(); ++i){

            if ((laserScan.ranges.at(i) > laserScan.range_min) &&
                (laserScan.ranges.at(i) < laserScan.range_max) &&
                !isnan(laserScan.ranges.at(i)) && isfinite(laserScan.ranges.at(i))){
                    
                objectIndex.push_back(i);
            }
        }

        for (unsigned int i = 1; i < objectIndex.size(); ++i){

            if(abs(laserScan.ranges.at(objectIndex.at(i)) - laserScan.ranges.at(objectIndex.at(i-1))) < 0.3){
                cluster.push_back(objectIndex.at(i));
            }
            else{
                segments.push_back(cluster);
                cluster.clear();
            }
        }
        return segments;
    }

    std::vector<geometry_msgs::msg::Point> getCircle(nav_msgs::msg::Odometry odo_){
        
        std::vector<int> index;
        std::vector<geometry_msgs::msg::Point> cones;
        std::vector<float> ranges = laserScan_.ranges;
        std::vector<std::vector<int>> segments = getSegments();         // updates segments

        for(auto cluster : segments){
            if(cluster.size() < 5){continue;}

            detectArc(cluster, circle_centers);
            
            
            if(dist < 1){
                index.push_back(cluster.at(cluster.size()/2));      // index of the middle of the segment
            } 
        }

        for(auto i : index){
            geometry_msgs::msg::Point point = polarToCart(i);
            Eigen::Vector3d local_point(point.x, point.y, 0);      
            // Pose of the car
            Eigen::Vector3d translation(odo_.pose.pose.position.x, odo_.pose.pose.position.y, 0);
            Eigen::Quaterniond rotation(odo_.pose.pose.orientation.w, odo_.pose.pose.orientation.x, 
                                        odo_.pose.pose.orientation.y, odo_.pose.pose.orientation.z);
            rotation.normalize();
            //Rotation Matrix
            Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
            Eigen::Vector3d world_point = rotation_matrix * local_point + translation;
            point.x = world_point.x();
            point.y = world_point.y();
            cones.push_back(point);
        }
        
        return cones;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    cv::Mat map_image_;         // map obtained from 'map/' node
    cv::Point circle_loc_;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::LaserScan laserScan_;

    double map_scale_;
    double origin_x_;
    double origin_y_;
    unsigned int size_x_;
    unsigned int size_y_;

    bool mapped = false;

    const float map_resolution = 0.05; // Resolution in meters per pixel (adjust as needed)

    const std::string WINDOW1 = "Circle Finder";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sprint3>());
    rclcpp::shutdown();
    return 0;
}