#include <chrono>
#include <memory>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp> // Added for imshow

// Logic uses the lightweight detection message
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"

using namespace std::chrono_literals;

class ArucoMissionNode : public rclcpp::Node {
public:
    ArucoMissionNode() : Node("aruco_mission_node") {
        // --- Parameters ---
        this->declare_parameter("target_marker_count", 5);
        this->declare_parameter("stop_distance", 1.0);
        this->declare_parameter("scan_speed", 0.4);
        this->declare_parameter("search_speed", 0.3);
        this->declare_parameter("backup_speed", -0.2);
        this->declare_parameter("backup_duration", 2.0);

        // --- Logic Subscriber ---
        detection_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
            "/aruco_detections", 10,
            std::bind(&ArucoMissionNode::detection_callback, this, std::placeholders::_1));

        // --- Visualization Subscriber ---
        // We need the raw image to draw the circle and show the window
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&ArucoMissionNode::image_callback, this, std::placeholders::_1));

        // --- Publisher ---
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // --- Timer ---
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&ArucoMissionNode::control_loop, this));

        // Setup ArUco for Visualization
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        parameters_ = cv::aruco::DetectorParameters::create();

        RCLCPP_INFO(this->get_logger(), "Mission Started. check the 'Robot View' window.");
    }

private:
    enum State { SCANNING, SORTING, NAVIGATING, BACKING_UP, FINISHED };
    State state_ = SCANNING;

    // Data
    std::set<int> found_ids_;
    std::vector<int> sorted_targets_;
    size_t current_target_idx_ = 0;
    
    std::map<int, geometry_msgs::msg::Point> visible_markers_;
    rclcpp::Time backup_start_time_;

    // ROS
    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // OpenCV
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    // ---------------------------------------------------------
    // 1. DETECTION CALLBACK (Logic)
    // ---------------------------------------------------------
    void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
        visible_markers_.clear();
        if (msg->markers.empty()) return;
            
        for (const auto &marker : msg->markers) {
            int id = marker.marker_id; 
            visible_markers_[id] = marker.pose.position;

                if (state_ == SCANNING) {
                    found_ids_.insert(id);
                }
            }
        }

    // ---------------------------------------------------------
    // 2. CONTROL LOOP (Movement)
    // ---------------------------------------------------------
    void control_loop() {
        geometry_msgs::msg::Twist cmd;
        int desired_count = this->get_parameter("target_marker_count").as_int();

        switch (state_) {
            case SCANNING:
                cmd.angular.z = this->get_parameter("scan_speed").as_double();
                if (found_ids_.size() >= (size_t)desired_count) {
                    state_ = SORTING;
                }
                break;

            case SORTING:
                cmd.linear.x = 0; cmd.angular.z = 0;
                sorted_targets_.assign(found_ids_.begin(), found_ids_.end());
                std::sort(sorted_targets_.begin(), sorted_targets_.end());
                state_ = NAVIGATING;
                break;

            case NAVIGATING:
                if (current_target_idx_ >= sorted_targets_.size()) {
                    state_ = FINISHED;
                    break;
                }
                
                {
                    int target_id = sorted_targets_[current_target_idx_];

                    if (visible_markers_.count(target_id)) {
                        // Servoing Logic
                        auto pos = visible_markers_[target_id];
                        double error_yaw = -pos.x; 
                        double error_dist = pos.z - this->get_parameter("stop_distance").as_double();

                        cmd.angular.z = 0.2 * error_yaw; 

                        if (std::abs(error_yaw) < 0.2) {
                            cmd.linear.x = std::min(0.5 * error_dist, 0.5);
                        }
                            
                        if (std::abs(error_dist) < 0.05) {
                             state_ = BACKING_UP;
                             backup_start_time_ = this->now();
                        }
                    } else {
                        cmd.angular.z = this->get_parameter("search_speed").as_double();
                    }
                }
                break;

            case BACKING_UP:
                {
                    double elapsed = (this->now() - backup_start_time_).seconds();
                    if (elapsed < this->get_parameter("backup_duration").as_double()) {
                        cmd.linear.x = this->get_parameter("backup_speed").as_double();
                        cmd.angular.z = 0.0;
                    } else {
                        cmd.linear.x = 0;
                        current_target_idx_++; 
                        state_ = NAVIGATING;    
                    }
                }
                break;

            case FINISHED:
                cmd.linear.x = 0; cmd.angular.z = 0;
                break;
        }
        vel_pub_->publish(cmd);
    }

    // ---------------------------------------------------------
    // 3. IMAGE CALLBACK (Visualization + ImShow)
    // ---------------------------------------------------------
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) { return; }

        // Local detection to find pixels for drawing
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);

        int target_id = -1;
        if (state_ == NAVIGATING && current_target_idx_ < sorted_targets_.size()) {
            target_id = sorted_targets_[current_target_idx_];
        }

        // Draw Logic
        for (size_t i = 0; i < ids.size(); ++i) {
            // Draw standard markers
            cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

            // Draw GREEN CIRCLE if this is the target
            if (ids[i] == target_id) {
                cv::Point2f center = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
                float area = cv::contourArea(corners[i]);
                float radius = std::sqrt(area) / 1.8f; 
                
                // Draw circle: Image, Center, Radius, Color(BGR), Thickness
                cv::circle(cv_ptr->image, center, (int)radius, cv::Scalar(0, 255, 0), 4);
                cv::putText(cv_ptr->image, "TARGET", center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            }
        }
        
        // Status Text
        std::string label = "WAITING";
        if (state_ == NAVIGATING) label = "SEEKING ID: " + std::to_string(target_id);
        else if (state_ == BACKING_UP) label = "BACKING UP";
        else if (state_ == FINISHED) label = "DONE";
        else if (state_ == SCANNING) label = "SCANNING";

        cv::putText(cv_ptr->image, label, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
        
        // --- DISPLAY WINDOW ---
        cv::imshow("Robot View", cv_ptr->image);
        cv::waitKey(1); // Essential for the window to update!
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMissionNode>());
    rclcpp::shutdown();
    return 0;
}