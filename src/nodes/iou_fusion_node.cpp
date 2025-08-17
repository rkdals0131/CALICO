#include "calico/nodes/iou_fusion_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <algorithm>
#include <numeric>

namespace calico {
namespace nodes {

IoUFusionNode::IoUFusionNode(const rclcpp::NodeOptions& options)
    : Node("iou_fusion_node", options),
      iou_threshold_(0.1),
      config_file_path_(""),
      camera_intrinsic_file_(""),
      camera_extrinsic_file_(""),
      lidar_box_color_(cv::Scalar(0, 255, 0)),     // Green for LiDAR
      yolo_box_color_(cv::Scalar(255, 0, 0)),      // Blue for YOLO
      matched_box_color_(cv::Scalar(0, 255, 255)), // Yellow for matches
      line_thickness_(2),
      font_face_(cv::FONT_HERSHEY_SIMPLEX),
      font_scale_(0.5)
{
    // Declare parameters
    this->declare_parameter("iou_threshold", 0.1);
    this->declare_parameter("config_folder", "/home/user1/ROS2_Workspace/ros2_ws/src/hungarian_association/config/");
    this->declare_parameter("camera_intrinsic_file", "camera_intrinsic_calibration.yaml");
    this->declare_parameter("camera_extrinsic_file", "camera_extrinsic_calibration.yaml");
    this->declare_parameter("lidar_topic", "/cone/lidar/box");
    this->declare_parameter("yolo_topic", "/detections");
    this->declare_parameter("camera_image_topic", "/usb_cam/image_raw");
    this->declare_parameter("output_topic", "/fused_cones");
    this->declare_parameter("debug_image_topic", "/debug/iou_fusion_overlay");
    this->declare_parameter("sync_queue_size", 10);
    this->declare_parameter("sync_slop", 0.1);

    // Get parameters
    iou_threshold_ = this->get_parameter("iou_threshold").as_double();
    config_file_path_ = this->get_parameter("config_folder").as_string();
    camera_intrinsic_file_ = this->get_parameter("camera_intrinsic_file").as_string();
    camera_extrinsic_file_ = this->get_parameter("camera_extrinsic_file").as_string();
    
    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    std::string yolo_topic = this->get_parameter("yolo_topic").as_string();
    std::string camera_image_topic = this->get_parameter("camera_image_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string debug_image_topic = this->get_parameter("debug_image_topic").as_string();
    
    int sync_queue_size = this->get_parameter("sync_queue_size").as_int();
    double sync_slop = this->get_parameter("sync_slop").as_double();

    RCLCPP_INFO(this->get_logger(), "IoU Fusion Node starting...");
    RCLCPP_INFO(this->get_logger(), "IoU threshold: %.3f", iou_threshold_);
    RCLCPP_INFO(this->get_logger(), "Config folder: %s", config_file_path_.c_str());

    // Load camera calibration
    try {
        loadCameraCalibration();
        RCLCPP_INFO(this->get_logger(), "Camera calibration loaded successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load camera calibration: %s", e.what());
        throw;
    }

    // Initialize Hungarian matcher
    hungarian_matcher_ = std::make_unique<fusion::HungarianMatcher>();

    // Create subscribers using message_filters for synchronization
    lidar_sub_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::BoundingBox3DArray>>(
        this, lidar_topic);
    yolo_sub_ = std::make_shared<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>>(
        this, yolo_topic);
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, camera_image_topic);

    // Create synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(sync_queue_size), *lidar_sub_, *yolo_sub_, *image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(
        static_cast<int64_t>(sync_slop * 1e9)));
    sync_->registerCallback(&IoUFusionNode::fusionCallback, this);

    // Create publishers
    fused_pub_ = this->create_publisher<custom_interface::msg::ModifiedFloat32MultiArray>(
        output_topic, 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        debug_image_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Subscribing to:");
    RCLCPP_INFO(this->get_logger(), "  LiDAR: %s", lidar_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  YOLO: %s", yolo_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera: %s", camera_image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to:");
    RCLCPP_INFO(this->get_logger(), "  Fused: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Debug: %s", debug_image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "IoU Fusion Node initialized successfully");
}

void IoUFusionNode::loadCameraCalibration()
{
    // Load camera intrinsic calibration
    std::string intrinsic_path = config_file_path_ + camera_intrinsic_file_;
    YAML::Node intrinsic_config = YAML::LoadFile(intrinsic_path);
    
    auto camera_matrix_data = intrinsic_config["camera_matrix"]["data"].as<std::vector<double>>();
    auto dist_coeffs_data = intrinsic_config["distortion_coefficients"]["data"].as<std::vector<double>>();
    
    camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
    dist_coeffs_ = cv::Mat(1, dist_coeffs_data.size(), CV_64F, dist_coeffs_data.data()).clone();

    // Load camera extrinsic calibration (LiDAR to camera transform)
    std::string extrinsic_path = config_file_path_ + camera_extrinsic_file_;
    YAML::Node extrinsic_config = YAML::LoadFile(extrinsic_path);
    
    auto extrinsic_matrix_data = extrinsic_config["extrinsic_matrix"].as<std::vector<double>>();
    
    // Convert to Eigen matrix
    T_lidar_to_cam_ = Eigen::Matrix4d::Zero();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_lidar_to_cam_(i, j) = extrinsic_matrix_data[i * 4 + j];
        }
    }

    RCLCPP_INFO(this->get_logger(), "Camera matrix:\n[%.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f]",
                camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(0,1), camera_matrix_.at<double>(0,2),
                camera_matrix_.at<double>(1,0), camera_matrix_.at<double>(1,1), camera_matrix_.at<double>(1,2),
                camera_matrix_.at<double>(2,0), camera_matrix_.at<double>(2,1), camera_matrix_.at<double>(2,2));
}

std::vector<cv::Point3f> IoUFusionNode::get3DBoundingBoxCorners(
    const vision_msgs::msg::BoundingBox3D& bbox)
{
    std::vector<cv::Point3f> corners;
    
    double cx = bbox.center.position.x;
    double cy = bbox.center.position.y;
    double cz = bbox.center.position.z;
    
    double dx = bbox.size.x / 2.0;
    double dy = bbox.size.y / 2.0;
    double dz = bbox.size.z / 2.0;

    // 8 corners of the AABB (Axis-Aligned Bounding Box)
    // Note: For traffic cones, we intentionally use AABB and ignore orientation
    // since cones are rotationally symmetric objects with no directional preference.
    // This simplifies projection calculations and is appropriate for cone detection.
    corners.emplace_back(cx + dx, cy + dy, cz - dz);  // Front-top-right
    corners.emplace_back(cx + dx, cy - dy, cz - dz);  // Front-bottom-right
    corners.emplace_back(cx - dx, cy - dy, cz - dz);  // Front-bottom-left
    corners.emplace_back(cx - dx, cy + dy, cz - dz);  // Front-top-left
    corners.emplace_back(cx + dx, cy + dy, cz + dz);  // Back-top-right
    corners.emplace_back(cx + dx, cy - dy, cz + dz);  // Back-bottom-right
    corners.emplace_back(cx - dx, cy - dy, cz + dz);  // Back-bottom-left
    corners.emplace_back(cx - dx, cy + dy, cz + dz);  // Back-top-left

    return corners;
}

cv::Rect2f IoUFusionNode::project3DBoxTo2D(const std::vector<cv::Point3f>& corners_3d)
{
    try {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(corners_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                         camera_matrix_, dist_coeffs_, projected_points);

        if (projected_points.empty()) {
            return cv::Rect2f(0, 0, 0, 0);
        }

        // Find min/max coordinates to create bounding rectangle
        float x_min = projected_points[0].x;
        float x_max = projected_points[0].x;
        float y_min = projected_points[0].y;
        float y_max = projected_points[0].y;

        for (const auto& pt : projected_points) {
            x_min = std::min(x_min, pt.x);
            x_max = std::max(x_max, pt.x);
            y_min = std::min(y_min, pt.y);
            y_max = std::max(y_max, pt.y);
        }

        return cv::Rect2f(x_min, y_min, x_max - x_min, y_max - y_min);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to project 3D box to 2D: %s", e.what());
        return cv::Rect2f(0, 0, 0, 0);
    }
}

double IoUFusionNode::calculateIoU(const cv::Rect2f& boxA, const cv::Rect2f& boxB)
{
    // Calculate intersection area
    float x_left = std::max(boxA.x, boxB.x);
    float y_top = std::max(boxA.y, boxB.y);
    float x_right = std::min(boxA.x + boxA.width, boxB.x + boxB.width);
    float y_bottom = std::min(boxA.y + boxA.height, boxB.y + boxB.height);

    if (x_right <= x_left || y_bottom <= y_top) {
        return 0.0;  // No intersection
    }

    float intersection_area = (x_right - x_left) * (y_bottom - y_top);
    float area_A = boxA.width * boxA.height;
    float area_B = boxB.width * boxB.height;
    float union_area = area_A + area_B - intersection_area;

    return union_area > 0 ? intersection_area / union_area : 0.0;
}

std::vector<std::pair<int, int>> IoUFusionNode::performHungarianMatching(
    const std::vector<cv::Rect2f>& projected_boxes,
    const std::vector<cv::Rect2f>& yolo_boxes)
{
    if (projected_boxes.empty() || yolo_boxes.empty()) {
        return {};
    }

    size_t n_projected = projected_boxes.size();
    size_t n_yolo = yolo_boxes.size();

    // Create cost matrix (cost = 1 - IoU)
    Eigen::MatrixXd cost_matrix(n_yolo, n_projected);
    
    for (size_t i = 0; i < n_yolo; ++i) {
        for (size_t j = 0; j < n_projected; ++j) {
            double iou = calculateIoU(yolo_boxes[i], projected_boxes[j]);
            cost_matrix(i, j) = 1.0 - iou;
        }
    }

    // Use Hungarian algorithm to find optimal matching
    auto match_result = hungarian_matcher_->match(cost_matrix, 1.0 - iou_threshold_);

    std::vector<std::pair<int, int>> valid_matches;
    for (const auto& match : match_result.matches) {
        int yolo_idx = match.first;   // detection_idx
        int lidar_idx = match.second; // track_idx
        
        double iou = 1.0 - cost_matrix(yolo_idx, lidar_idx);
        if (iou >= iou_threshold_) {
            valid_matches.emplace_back(yolo_idx, lidar_idx);
        }
    }

    return valid_matches;
}

cv::Mat IoUFusionNode::createDebugVisualization(
    const cv::Mat& image,
    const std::vector<cv::Rect2f>& projected_boxes,
    const std::vector<cv::Rect2f>& yolo_boxes,
    const std::vector<std::pair<int, int>>& matches,
    const std::vector<double>& iou_scores)
{
    cv::Mat debug_image = image.clone();

    // Draw projected LiDAR boxes (green)
    for (size_t i = 0; i < projected_boxes.size(); ++i) {
        const auto& box = projected_boxes[i];
        cv::rectangle(debug_image, box, lidar_box_color_, line_thickness_);
        
        std::string label = "L" + std::to_string(i);
        cv::putText(debug_image, label, 
                   cv::Point(box.x, box.y - 5), 
                   font_face_, font_scale_, lidar_box_color_, 1);
    }

    // Draw YOLO boxes (blue)
    for (size_t i = 0; i < yolo_boxes.size(); ++i) {
        const auto& box = yolo_boxes[i];
        cv::rectangle(debug_image, box, yolo_box_color_, line_thickness_);
        
        std::string label = "Y" + std::to_string(i);
        cv::putText(debug_image, label, 
                   cv::Point(box.x + box.width - 30, box.y - 5), 
                   font_face_, font_scale_, yolo_box_color_, 1);
    }

    // Highlight matched pairs and show IoU scores
    for (size_t i = 0; i < matches.size() && i < iou_scores.size(); ++i) {
        int yolo_idx = matches[i].first;
        int lidar_idx = matches[i].second;
        
        const auto& yolo_box = yolo_boxes[yolo_idx];
        const auto& lidar_box = projected_boxes[lidar_idx];
        
        // Draw thick yellow border for matched boxes
        cv::rectangle(debug_image, yolo_box, matched_box_color_, line_thickness_ + 1);
        cv::rectangle(debug_image, lidar_box, matched_box_color_, line_thickness_ + 1);
        
        // Draw line connecting matched centers
        cv::Point2f yolo_center(yolo_box.x + yolo_box.width/2, yolo_box.y + yolo_box.height/2);
        cv::Point2f lidar_center(lidar_box.x + lidar_box.width/2, lidar_box.y + lidar_box.height/2);
        cv::line(debug_image, yolo_center, lidar_center, matched_box_color_, 2);
        
        // Show IoU score
        std::string iou_text = "IoU: " + std::to_string(iou_scores[i]).substr(0, 4);
        cv::Point text_pos((yolo_center.x + lidar_center.x) / 2, 
                          (yolo_center.y + lidar_center.y) / 2);
        cv::putText(debug_image, iou_text, text_pos, 
                   font_face_, font_scale_, matched_box_color_, 2);
    }

    // Add legend
    int legend_y = 30;
    cv::putText(debug_image, "Green: LiDAR projected", cv::Point(10, legend_y), 
               font_face_, font_scale_, lidar_box_color_, 2);
    cv::putText(debug_image, "Blue: YOLO detections", cv::Point(10, legend_y + 25), 
               font_face_, font_scale_, yolo_box_color_, 2);
    cv::putText(debug_image, "Yellow: Matched pairs", cv::Point(10, legend_y + 50), 
               font_face_, font_scale_, matched_box_color_, 2);

    return debug_image;
}

custom_interface::msg::ModifiedFloat32MultiArray IoUFusionNode::createFusedMessage(
    const vision_msgs::msg::BoundingBox3DArray::SharedPtr& lidar_msg,
    const yolo_msgs::msg::DetectionArray::SharedPtr& yolo_msg,
    const std::vector<std::pair<int, int>>& matches)
{
    custom_interface::msg::ModifiedFloat32MultiArray fused_msg;
    fused_msg.header = lidar_msg->header;

    // Initialize all as "Unknown"
    std::vector<std::string> class_names(lidar_msg->boxes.size(), "Unknown");
    
    // Update class names based on matches
    for (const auto& match : matches) {
        int yolo_idx = match.first;
        int lidar_idx = match.second;
        
        if (yolo_idx < static_cast<int>(yolo_msg->detections.size()) && 
            lidar_idx < static_cast<int>(lidar_msg->boxes.size())) {
            class_names[lidar_idx] = yolo_msg->detections[yolo_idx].class_name;
        }
    }

    // Extract 3D coordinates
    std::vector<float> data;
    for (const auto& box : lidar_msg->boxes) {
        data.push_back(static_cast<float>(box.center.position.x));
        data.push_back(static_cast<float>(box.center.position.y));
        data.push_back(static_cast<float>(box.center.position.z));
    }

    fused_msg.data = data;
    fused_msg.class_names = class_names;

    // Set layout
    size_t num_cones = lidar_msg->boxes.size();
    if (num_cones > 0) {
        std_msgs::msg::MultiArrayDimension dim1, dim2;
        dim1.label = "cones";
        dim1.size = num_cones;
        dim1.stride = num_cones * 3;
        
        dim2.label = "coords";
        dim2.size = 3;
        dim2.stride = 3;
        
        fused_msg.layout.dim.push_back(dim1);
        fused_msg.layout.dim.push_back(dim2);
    }

    return fused_msg;
}

void IoUFusionNode::fusionCallback(
    const vision_msgs::msg::BoundingBox3DArray::SharedPtr lidar_msg,
    const yolo_msgs::msg::DetectionArray::SharedPtr yolo_msg,
    const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    try {
        // Convert YOLO detections to cv::Rect2f format
        std::vector<cv::Rect2f> yolo_boxes;
        for (const auto& detection : yolo_msg->detections) {
            float cx = detection.bbox.center.position.x;
            float cy = detection.bbox.center.position.y;
            float w = detection.bbox.size.x;
            float h = detection.bbox.size.y;
            
            if (w > 0 && h > 0) {
                float x = cx - w/2;
                float y = cy - h/2;
                yolo_boxes.emplace_back(x, y, w, h);
            }
        }

        // Project LiDAR 3D boxes to 2D
        std::vector<cv::Rect2f> projected_boxes;
        for (const auto& bbox_3d : lidar_msg->boxes) {
            auto corners_3d = get3DBoundingBoxCorners(bbox_3d);
            auto box_2d = project3DBoxTo2D(corners_3d);
            
            if (box_2d.width > 0 && box_2d.height > 0) {
                projected_boxes.push_back(box_2d);
            }
        }

        // Perform Hungarian matching
        auto matches = performHungarianMatching(projected_boxes, yolo_boxes);

        // Calculate IoU scores for matched pairs
        std::vector<double> iou_scores;
        for (const auto& match : matches) {
            if (match.first < static_cast<int>(yolo_boxes.size()) && 
                match.second < static_cast<int>(projected_boxes.size())) {
                double iou = calculateIoU(yolo_boxes[match.first], projected_boxes[match.second]);
                iou_scores.push_back(iou);
            }
        }

        // Create and publish fused message
        auto fused_msg = createFusedMessage(lidar_msg, yolo_msg, matches);
        fused_pub_->publish(fused_msg);

        // Create and publish debug visualization
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat debug_image = createDebugVisualization(
            cv_ptr->image, projected_boxes, yolo_boxes, matches, iou_scores);
        
        auto debug_msg = cv_bridge::CvImage(image_msg->header, "bgr8", debug_image).toImageMsg();
        debug_image_pub_->publish(*debug_msg);

        RCLCPP_INFO(this->get_logger(), 
                   "Processed fusion: %zu LiDAR boxes, %zu YOLO detections, %zu matches",
                   lidar_msg->boxes.size(), yolo_msg->detections.size(), matches.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in fusion callback: %s", e.what());
    }
}

} // namespace nodes
} // namespace calico

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(calico::nodes::IoUFusionNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<calico::nodes::IoUFusionNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}