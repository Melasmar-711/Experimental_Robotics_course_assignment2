#include <chrono>
#include <memory>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"      

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

class ArucoMissionNode : public rclcpp::Node {
public:
    ArucoMissionNode() : Node("aruco_mission_node") {
        // --- Parameters ---
        this->declare_parameter("target_marker_count", 5);
        this->declare_parameter("stop_distance", 0.4);
        this->declare_parameter("scan_speed", 0.4);
        this->declare_parameter("search_speed", 0.1);
        
        // FIX 1: Backup speed should be negative to actually go backward
        this->declare_parameter("backup_speed", 0.0); 
        this->declare_parameter("backup_duration", 2.0);
        this->declare_parameter("marker_size", 0.05); 

        // --- Camera Info Subscriber ---
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/rgb/camera_info", 10,
            std::bind(&ArucoMissionNode::info_callback, this, std::placeholders::_1));

        // --- Compressed Image Subscriber ---
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/rgb/image_raw/compressed", 10,
            std::bind(&ArucoMissionNode::compressed_callback, this, std::placeholders::_1));

        // --- Publisher ---
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // --- Timer ---
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&ArucoMissionNode::control_loop, this));

        // Setup ArUco 
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        parameters_ = cv::aruco::DetectorParameters::create();

        RCLCPP_INFO(this->get_logger(), "Node Started. Filtering out ID 0 (Noise).");
    }

private:
    enum State { SCANNING, SORTING, NAVIGATING, BACKING_UP, FINISHED };
    State state_ = SCANNING;

    // Logic Data
    std::set<int> found_ids_;
    std::vector<int> sorted_targets_;
    size_t current_target_idx_ = 0;
    
    // Position Data
    std::map<int, geometry_msgs::msg::Point> visible_markers_;
    rclcpp::Time backup_start_time_;

    // Camera Data
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_ = false;

    // ROS pointers
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // OpenCV pointers
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    // ---------------------------------------------------------
    // 1. CAMERA INFO CALLBACK
    // ---------------------------------------------------------
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (camera_info_received_) return;

        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                camera_matrix_.at<double>(i, j) = msg->k[i*3+j];
            }
        }

        dist_coeffs_ = cv::Mat(1, 5, CV_64F);
        for(size_t i=0; i<msg->d.size() && i<5; i++) {
            dist_coeffs_.at<double>(0, i) = msg->d[i];
        }

        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera Intrinsics Configured.");
    }

    // ---------------------------------------------------------
    // 2. COMPRESSED IMAGE CALLBACK
    // ---------------------------------------------------------
    void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (!camera_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);

        visible_markers_.clear();

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            double marker_len = this->get_parameter("marker_size").as_double();
            
            cv::aruco::estimatePoseSingleMarkers(corners, marker_len, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                int id = ids[i];
                
                if (id == 0) continue; // Noise filter

                geometry_msgs::msg::Point p;
                p.x = tvecs[i][0]; 
                p.y = tvecs[i][1]; 
                p.z = tvecs[i][2]; 
                visible_markers_[id] = p;

                if (state_ == SCANNING) {
                    found_ids_.insert(id);
                }
            }
        }

        // --- VISUALIZATION ---
        int target_id = -1;
        if (state_ == NAVIGATING && current_target_idx_ < sorted_targets_.size()) {
            target_id = sorted_targets_[current_target_idx_];
        }

        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] == 0) continue;

            if (ids[i] == target_id) {
                cv::Point2f center = (corners[i][0] + corners[i][2]) * 0.5f;
                float radius = 80.0f;
                cv::circle(cv_ptr->image, center, (int)radius, cv::Scalar(0, 255, 0), 4);
            }
        }
        
        std::string status = "WAITING";
        if (state_ == SCANNING) status = "SCANNING";
        else if (state_ == SORTING) status = "SORTING";
        else if (state_ == NAVIGATING) status = "TRACKING ID: " + std::to_string(target_id);
        else if (state_ == BACKING_UP) status = "BACKING UP";
        else if (state_ == FINISHED) status = "FINISHED";

        cv::putText(cv_ptr->image, status, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 3);
        
        cv::imshow("Robot Compressed View", cv_ptr->image);
        cv::waitKey(1);
    }

    // ---------------------------------------------------------
    // 3. CONTROL LOOP
    // ---------------------------------------------------------
    void control_loop() {
        if (!camera_info_received_) return; 

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
                        auto pos = visible_markers_[target_id];
                        double error_yaw = -pos.x; 
                        double error_dist = pos.z - this->get_parameter("stop_distance").as_double();

                        cmd.angular.z = 0.2 * error_yaw; 

                        // FIX 2: Use POSITIVE velocity to move forward (approach)
                        if (std::abs(error_yaw) < 0.01) {
                            cmd.linear.x = 0; 
                            
                            if (std::abs(error_dist) < 10) {
                             state_ = BACKING_UP;
                             backup_start_time_ = this->now();
                        }
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
                        // This will be negative now (-0.2) based on the parameter fix above
                        cmd.linear.x = this->get_parameter("backup_speed").as_double();
                        cmd.angular.z = 0.0;
                    } else {
                        cmd.linear.x = 0;
                        std::this_thread::sleep_for(std::chrono::seconds(10)); // Pause for 10 seconds to simulate action
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMissionNode>());
    rclcpp::shutdown();
    return 0;
}
