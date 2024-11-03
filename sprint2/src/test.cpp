#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode() : Node("map_processor_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapProcessorNode::odom_callback, this, std::placeholders::_1));

        XYpublisher_ = this->create_publisher<geometry_msgs::msg::Point>("/xy_point", 10);

        // cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW2, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW3, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW4, cv::WINDOW_AUTOSIZE);

    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        laserScanToCvMat(msg);
        if(mapped){
            MakeLocaliseImage();
        }
        std::cout << "laser scan received" << std::endl;
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        occupancyGridToImage(mapMsg);
        tmp_col_img = m_MapColImage.clone();
        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);

        // cv::imshow(WINDOW1, tmp_col_img);
        // cv::waitKey(1);

        mapped = true;
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position data
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        std::cout << "Odom Point: (" << x << ", " << 
                                y << ")" << std::endl;

        auto point_msg = geometry_msgs::msg::Point();
        
        // Set the x and y values (you can modify these as needed)
        point_msg.x = x; // Example x value
        point_msg.y = y; // Example y value
        point_msg.z = 0.0; // Set z to 0 if only x and y are used
        XYpublisher_->publish(point_msg);
    }

    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

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
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) <<  0, 0, 0,
                                                    0, 1, 0,
                                                    0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);

        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        std::cout << "Occupancy grid map converted to a binary image\n";
    }

    void laserScanToCvMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
    {
        // Create a blank image (white background)
        image = cv::Mat::zeros(image_size, image_size, CV_8UC1);
        image.setTo(cv::Scalar(0)); // Set background to white

        // Calculate center of the image (robot's position)
        int center_x = image_size / 2;
        int center_y = image_size / 2;

        // Loop through each range measurement in the LaserScan
        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            // Extract range and angle
            float range = scan->ranges[i];
            float angle = scan->angle_min + i * scan->angle_increment + 1.5708;

            // Ignore invalid ranges
            if (std::isinf(range) || std::isnan(range))
                continue;

            // Convert polar coordinates (range, angle) to Cartesian (x, y)
            float x = range * cos(angle);
            float y = range * sin(angle);

            // Convert world coordinates (x, y) to pixel coordinates
            int pixel_x = static_cast<int>(center_x + x / map_resolution);
            int pixel_y = static_cast<int>(center_y - y / map_resolution); // Invert y-axis for image coordinates

            // Ensure points are within image bounds
            if (pixel_x >= 0 && pixel_x < image_size && pixel_y >= 0 && pixel_y < image_size)
            {
                // Set the pixel to black (occupied)
                image.at<uchar>(pixel_y, pixel_x) = 255;
            }
        }
    }

    void MakeLocaliseImage(void){
        cv::Mat localisedImage = tmp_col_img.clone();
        cv::Mat scan_image = image.clone();
        
        // Convert images to grayscale if they are not already
        if (localisedImage.channels() > 1){
            cv::cvtColor(localisedImage, localisedImage, cv::COLOR_BGR2GRAY);
        }
        if (scan_image.channels() > 1){
            cv::cvtColor(scan_image, scan_image, cv::COLOR_BGR2GRAY);
        }
            
        // Ensure both images are 8-bit (CV_8U)
        if (localisedImage.type() != CV_8U){
            localisedImage.convertTo(localisedImage, CV_8U);
        }
        if (scan_image.type() != CV_8U){
            scan_image.convertTo(scan_image, CV_8U);
        }
        
        cv::GaussianBlur(localisedImage, localisedImage, cv::Size(3, 3), 0);
        cv::GaussianBlur(scan_image, scan_image, cv::Size(3, 3), 0);
        // used for the template match
        cv::Canny(localisedImage, localisedImage, 50, 150);
        cv::Canny(scan_image, scan_image, 50, 150);

        if (!localisedImage.empty() && !scan_image.empty()) {
            // Find the robot location in the map
            
            cv::Point robot_location = findRobotLocation(localisedImage, scan_image);
            //OR 
            // cv::Point robot_location = calculateYawChange(localisedImage, scan_image);

            m_MapColImage2.create(localisedImage.size(), CV_8UC3);
            cv::cvtColor(localisedImage, m_MapColImage2, cv::COLOR_GRAY2BGR);            

            // Draw a circle around the found location in the MAP image
            cv::circle(m_MapColImage2, robot_location, 5, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow(WINDOW3, m_MapColImage2);
        cv::imshow(WINDOW2, scan_image);
        cv::waitKey(1); // Small delay to allow the image to update
    }

    // using match template
    cv::Point findRobotLocation(const cv::Mat& map_image, const cv::Mat& scan_image)
    {
        std::vector<double> maxValVec;
        // Get image dimensions
        int width = scan_image.cols;
        int height = scan_image.rows;
        cv::Mat result;

        // Calculate the center of the image
        cv::Point2f center(width / 2.0, height / 2.0);

        for(double angle_degrees = -180; angle_degrees < 180; angle_degrees++){            
            // Result image where we store the match result

            cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle_degrees, 1.0);
            
            // Determine the bounding rectangle after the rotation
            cv::Rect bbox = cv::RotatedRect(center, scan_image.size(), angle_degrees).boundingRect();

            // Adjust the rotation matrix to take into account the translation
            rotation_matrix.at<double>(0, 2) += bbox.width / 2.0 - center.x;
            rotation_matrix.at<double>(1, 2) += bbox.height / 2.0 - center.y;

            // Perform the rotation
            cv::Mat rotated_image;
            cv::warpAffine(scan_image, rotated_image, rotation_matrix, bbox.size());

            // Perform template matching (TM_CCOEFF_NORMED is a common choice)
            // cv::matchTemplate(map_image, scan_image, result, cv::TM_CCOEFF_NORMED);
            cv::matchTemplate(map_image, rotated_image, result, cv::TM_CCOEFF_NORMED);
            
            // Find the best match location
            double minVal, maxVal;
            maxValVec.push_back(maxVal); 
        }
        double minVal = 0;
        double maxVal = 0;
        for(auto loc : maxValVec){
            if(loc > maxVal){
                maxVal = loc;
            }
        }

        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

        maxLoc.x += width/2;
        maxLoc.y += height/2;
        double y = (map_image.cols/2 - maxLoc.x) * map_resolution;
        double x = (map_image.rows/2 - maxLoc.y) * map_resolution;

        std::cout << "Translation Point: (" << maxLoc.x << ", " << 
                                maxLoc.y << ")" << std::endl;
        std::cout << "Coordinate Point: (" << x << ", " << 
                                y << ")" << std::endl;

        // Return the location of the best match (maxLoc)
        return maxLoc;
    }

    cv::Point calculateYawChange(const cv::Mat& first_image_, const cv::Mat& second_image_) {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 10 || dstPoints.size() < 10) { // point size for accuracy
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            cv::Point temp(-1, -1);
            return(temp);
        }

        try {
            transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                float tx = transform_matrix.at<float>(0, 2); // tx = 30
                float ty = transform_matrix.at<float>(1, 2); // ty = 50

                // Create a cv::Point2f from the translation
                cv::Point translationPoint(tx, ty);
                translationPoint.x += m_MapColImage2.cols/2;
                translationPoint.y += m_MapColImage2.rows/2;
                std::cout << "Translation Point: (" << translationPoint.x << ", " << 
                                    translationPoint.y << ")" << std::endl;
                return translationPoint;
                // std::cout << "Translation Point: (" << translationPoint.x << ", " << 
                //                 translationPoint.y << ")" << std::endl;
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }

        // Draw matches and visualize
        cv::Mat matchImage;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, matchImage,
                        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Display the result
        cv::imshow(WINDOW4, matchImage);
        cv::waitKey(1);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr XYpublisher_;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    cv::Mat m_MapColImage2;
    cv::Mat image;
    cv::Mat tmp_col_img;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;
    bool mapped = false;
    double angle_difference_;
    cv::Mat transform_matrix;

    const int image_size = 120; // Size of the image (500x500 pixels)
    const float map_resolution = 0.05; // Resolution in meters per pixel (adjust as needed)

    const std::string WINDOW1 = "MAP";
    const std::string WINDOW2 = "SCAN";
    const std::string WINDOW3 = "LOCALISED";
    const std::string WINDOW4 = "MATCHING";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}