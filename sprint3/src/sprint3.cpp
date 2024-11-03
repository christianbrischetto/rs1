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
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>

class sprint3 : public rclcpp::Node
{
public:
    sprint3() : Node("cylinder_detector")
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&sprint3::scanCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&sprint3::odom_callback, this, std::placeholders::_1));

        cylinder_diameter_ = 0.3;
        odom_found_ = false;
        radius_tolerance = 0.2;

        // for testing purposes 
        // can open rviz, select add new, marker, select topic below
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        laserScan_ = *msg;
        if(odom_found_){
            std::vector<std::pair<double, double>> circle_centers = getCircle(odom_);
            
            std::cout << "\n---------- FOUND CIRCLES ----------" << std::endl;
            for(auto circle : circle_centers){
                std::cout << "x: " << circle.first << ", y: " << circle.second << std::endl;
            }
        publishCircleCenters(circle_centers);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position data
        odom_ = *msg;

        odom_found_ = true;
    }

    std::vector<std::pair<double, double>> getCircle(nav_msgs::msg::Odometry odo_){
        
        std::vector<std::pair<double, double>> circle_centers;
        std::vector<std::pair<double, double>> circle_centers_world;
        std::vector<std::vector<std::pair<double, double>>> segments = getSegments();         // updates segments

        for(auto cluster : segments){
            if(cluster.size() > 5){
                detectArc(cluster, circle_centers);
            }
        }

    // transform into world coords
        for(auto circle : circle_centers){
            Eigen::Vector3d local_point(circle.first, circle.second, 0);      
            // Pose of the car
            Eigen::Vector3d translation(odo_.pose.pose.position.x, odo_.pose.pose.position.y, 0);
            Eigen::Quaterniond rotation(odo_.pose.pose.orientation.w, odo_.pose.pose.orientation.x, 
                                        odo_.pose.pose.orientation.y, odo_.pose.pose.orientation.z);
            rotation.normalize();
            //Rotation Matrix
            Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
            Eigen::Vector3d world_point = rotation_matrix * local_point + translation;
            double x = world_point.x();
            double y = world_point.y();
            circle_centers_world.push_back({x, y});
        }
        
        return circle_centers_world;
    }

    // convert polar coordinates (distance and angle) to cartesian coords in the frame of the robot
    std::pair<double, double> polarToCart(unsigned int index)
    {
        float angle = laserScan_.angle_min + laserScan_.angle_increment*index; // + 1.5708;
        float range = laserScan_.ranges.at(index);
        geometry_msgs::msg::Point cart;
        double x = static_cast<double>(range*cos(angle));
        double y = static_cast<double>(range*sin(angle));
        std::pair<double, double> pair(x,y);
        return pair;
    }

    int calculateDistance(std::pair<double, double> p1, std::pair<double, double>p2){
        int dist = std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
        return dist;
    }

    std::vector<std::vector<std::pair<double, double>>> getSegments()
    {
        std::vector<std::vector<std::pair<double, double>>> segments; // vector of clusters
        std::vector<std::pair<double, double>> cluster;               // vector of indexes for laserScan range values within a single cluster/group
        std::vector<float> ranges = laserScan_.ranges;

        sensor_msgs::msg::LaserScan laserScan = laserScan_;   

        // obtains only valid readings
        std::vector<std::pair<double, double>> points;
        for (unsigned int i = 0; i < laserScan.ranges.size(); ++i){

            if ((laserScan.ranges.at(i) > laserScan.range_min) &&
                (laserScan.ranges.at(i) < laserScan.range_max) &&
                !isnan(laserScan.ranges.at(i)) && isfinite(laserScan.ranges.at(i))){

                std::pair<double, double> pair = polarToCart(i);   
                points.push_back(pair);
            }
        }

        for (unsigned int i = 1; i < points.size(); ++i){

            if((calculateDistance(points[i], points[i-1])) < 0.3){
                cluster.push_back(points[i]);
            }
            else{
                segments.push_back(cluster);
                cluster.clear();
            }
        }
        return segments;
    }

    /**
     * @brief determines wheter or not a cluster of points from a laser scan is in an arc and if so where the centre of the arc is located.
     * @param cluster the input cluster of points.
     * @param circle_centers the output estemation for the circles centre.
     */
    void detectArc(const std::vector<std::pair<double, double>> &cluster, std::vector<std::pair<double, double>> &circle_centers)
    {
        if (cluster.size() < 5)
        {
            return; // We need at least 5 points to verify the arc condition
        }

        const double target_radius = cylinder_diameter_ / 2.0;
        std::vector<std::pair<double, double>> potential_centers;
        std::vector<double> radii;

        // Iterate through combinations of points in the cluster
        for (size_t i = 0; i < cluster.size() - 2; ++i)
        {
            for (size_t j = i + 1; j < cluster.size() - 1; ++j)
            {
                for (size_t k = j + 1; k < cluster.size(); ++k)
                {
                    std::pair<double, double> center;
                    double radius;

                    if (calculateCircleFromThreePoints(cluster.at(i), cluster.at(j), cluster.at(k), radius, center))
                    {
                        potential_centers.push_back(center);
                        radii.push_back(radius);
                    }
                }
            }
        }

        // Calculate the mean center and radius from all potential circles
        if (!potential_centers.empty())
        {
            double avg_x = 0.0, avg_y = 0.0, avg_radius = 0.0;
            for (size_t i = 0; i < potential_centers.size(); ++i)
            {
                avg_x += potential_centers[i].first;
                avg_y += potential_centers[i].second;
                avg_radius += radii[i];
            }
            avg_x /= potential_centers.size();
            avg_y /= potential_centers.size();
            avg_radius /= potential_centers.size();

            // Check if the average radius is within the tolerance range
            if (std::abs(avg_radius - target_radius) <= radius_tolerance)
            {
                circle_centers.push_back({avg_x, avg_y});
            }
        }
    }

    /**
     * @brief Calculates the center and radius of a circle from three given points.
     * This function determines the circle that passes through three non-collinear points
     * by calculating the circle's center and radius. If the points are collinear,
     * the function returns `false`, as no circle can be formed. https://math.stackexchange.com/questions/213658/get-the-equation-of-a-circle-when-given-3-points
     *
     * @param p1 A pair representing the coordinates (x, y) of the first point.
     * @param p2 A pair representing the coordinates (x, y) of the second point.
     * @param p3 A pair representing the coordinates (x, y) of the third point.
     * @param[out] radius A reference to a double that will store the calculated radius of the circle.
     * @param[out] center A reference to a pair that will store the calculated center (x, y) of the circle.
     *
     * @return `true` if the circle was successfully calculated, `false` if the points are collinear.
     */
    bool calculateCircleFromThreePoints(const std::pair<double, double> &p1,
                                        const std::pair<double, double> &p2,
                                        const std::pair<double, double> &p3,
                                        double &radius,
                                        std::pair<double, double> &center)
    {
        double tol = 0.05;
        double x1 = p1.first, y1 = p1.second;
        double x2 = p2.first, y2 = p2.second;
        double x3 = p3.first, y3 = p3.second;

        double ma = (y2 - y1) / (x2 - x1);
        double mb = (y3 - y2) / (x3 - x2);

        // Check for collinearity (parallel slopes)
        if (std::abs(ma - mb) < tol)
        {
            return false; // The points are collinear, can't form a circle
        }

        // Calculate center of the circle
        double cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
        double cy = -1 / ma * (cx - (x1 + x2) / 2) + (y1 + y2) / 2;

        center = {cx, cy};
        radius = std::sqrt(std::pow(cx - x1, 2) + std::pow(cy - y1, 2));

        return true;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::LaserScan laserScan_;

    bool odom_found_;
    double cylinder_diameter_;
    double radius_tolerance; // Allowable deviation from the target radius

    void publishCircleCenters(const std::vector<std::pair<double, double>>& circle_centers) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";  // Ensure the frame matches your system
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "circle_centers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale (size) of the points
        marker.scale.x = 0.2;  // Set this to the desired size of the points in RViz
        marker.scale.y = 0.2;

        // Set the color of the points (RGBA)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;  // Alpha (transparency)

        // Fill the points with the circle centers
        for (const auto& center : circle_centers) {
            geometry_msgs::msg::Point p;
            p.x = center.first;  // x-coordinate
            p.y = center.second; // y-coordinate
            p.z = 0.0;  // Assuming a 2D plane, set z to 0
            marker.points.push_back(p);
        }

        // Publish the marker
        marker_pub_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sprint3>());
    rclcpp::shutdown();
    return 0;
}