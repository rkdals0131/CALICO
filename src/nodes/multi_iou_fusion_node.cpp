/**
 * @file multi_iou_fusion_node.cpp
 * @brief Multi-camera IoU-based sensor fusion node for ROS2
 * 
 * This node performs sensor fusion between LiDAR 3D bounding boxes and 
 * YOLO detections from multiple cameras using IoU-based Hungarian matching.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <custom_interface/msg/modified_float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include "calico/fusion/hungarian_matcher.hpp"
#include "calico/utils/config_loader.hpp"
#include <std_msgs/msg/multi_array_dimension.hpp>

namespace calico {
namespace nodes {

using BoundingBox3DArray = vision_msgs::msg::BoundingBox3DArray;
using DetectionArray = yolo_msgs::msg::DetectionArray;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using ModifiedFloat32MultiArray = custom_interface::msg::ModifiedFloat32MultiArray;

class MultiIoUFusionNode : public rclcpp::Node
{
public:
    explicit MultiIoUFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("multi_iou_fusion_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Multi-Camera IoU Fusion Node");
        
        // Declare parameters
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<double>("iou_threshold", 0.1);
        this->declare_parameter<bool>("enable_debug_viz", true);
        
        // Load configuration
        std::string config_file = this->get_parameter("config_file").as_string();
        if (config_file.empty()) {
            // Use default multi_hungarian_config.yaml
            config_file = "/home/user1/ROS2_Workspace/ros2_ws/src/hungarian_association/config/multi_hungarian_config.yaml";
            RCLCPP_WARN(this->get_logger(), "No config file specified, using default: %s", config_file.c_str());
        }
        
        loadConfiguration(config_file);
        
        // Setup publishers and subscribers
        setupPublishers();
        setupSubscribers();
        
        RCLCPP_INFO(this->get_logger(), "Multi-Camera IoU Fusion Node initialized with %zu cameras", 
                    camera_configs_.size());
    }

private:
    struct CameraConfig {
        std::string id;
        std::string detections_topic;
        std::string image_topic;
        std::string debug_image_topic;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        Eigen::Matrix4d T_lidar_to_cam;
        image_geometry::PinholeCameraModel cam_model;
    };
    
    // Configuration
    std::vector<CameraConfig> camera_configs_;
    std::string lidar_boxes_topic_;
    std::string fused_output_topic_;
    double iou_threshold_;
    bool enable_debug_viz_;
    int sync_queue_size_;
    double sync_slop_;
    
    // Publishers
    rclcpp::Publisher<ModifiedFloat32MultiArray>::SharedPtr fused_pub_;
    std::vector<rclcpp::Publisher<Image>::SharedPtr> debug_image_pubs_;
    
    // Subscribers and synchronizers
    std::shared_ptr<message_filters::Subscriber<BoundingBox3DArray>> lidar_sub_;
    std::vector<std::shared_ptr<message_filters::Subscriber<DetectionArray>>> detection_subs_;
    std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> image_subs_;
    
    // For 2 cameras (most common case)
    using SyncPolicy2 = message_filters::sync_policies::ApproximateTime<
        BoundingBox3DArray, DetectionArray, DetectionArray, Image, Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy2>> sync_2_;
    
    // Hungarian matcher
    std::unique_ptr<fusion::HungarianMatcher> matcher_;
    
    void loadConfiguration(const std::string& config_file)
    {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            auto ha_config = config["hungarian_association"];
            
            // Get topic names
            lidar_boxes_topic_ = ha_config["cones_topic"].as<std::string>();
            if (lidar_boxes_topic_ == "/sorted_cones_time") {
                // Override for new BoundingBox3D topic
                lidar_boxes_topic_ = "/cone/lidar/box";
                RCLCPP_INFO(this->get_logger(), "Using BoundingBox3D topic: %s", lidar_boxes_topic_.c_str());
            }
            fused_output_topic_ = ha_config["output_topic"].as<std::string>();
            
            // Get sync parameters
            auto qos_config = ha_config["qos"];
            sync_queue_size_ = qos_config["sync_queue_size"].as<int>(10);
            sync_slop_ = qos_config["sync_slop"].as<double>(0.1);
            
            // Load camera configurations
            auto cameras = ha_config["cameras"];
            auto calib_config = ha_config["calibration"];
            std::string config_folder = calib_config["config_folder"].as<std::string>();
            
            // Load intrinsic and extrinsic calibration files
            std::string intrinsic_file = config_folder + calib_config["camera_intrinsic_calibration"].as<std::string>();
            std::string extrinsic_file = config_folder + calib_config["camera_extrinsic_calibration"].as<std::string>();
            
            YAML::Node intrinsic_config = YAML::LoadFile(intrinsic_file);
            YAML::Node extrinsic_config = YAML::LoadFile(extrinsic_file);
            
            for (const auto& cam : cameras) {
                CameraConfig camera_cfg;
                camera_cfg.id = cam["id"].as<std::string>();
                camera_cfg.detections_topic = cam["detections_topic"].as<std::string>();
                
                // Get image topic (may not be in config)
                if (cam["image_topic"]) {
                    camera_cfg.image_topic = cam["image_topic"].as<std::string>();
                } else {
                    // Infer from camera ID
                    if (camera_cfg.id == "camera_1") {
                        camera_cfg.image_topic = "/usb_cam_1/image_raw";
                    } else if (camera_cfg.id == "camera_2") {
                        camera_cfg.image_topic = "/usb_cam_2/image_raw";
                    }
                }
                
                camera_cfg.debug_image_topic = "/debug/" + camera_cfg.id + "/iou_overlay";
                
                // Load calibration for this camera
                loadCameraCalibration(camera_cfg, intrinsic_config, extrinsic_config);
                
                camera_configs_.push_back(camera_cfg);
                RCLCPP_INFO(this->get_logger(), "Configured camera: %s", camera_cfg.id.c_str());
            }
            
            // Initialize matcher
            matcher_ = std::make_unique<fusion::HungarianMatcher>();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load configuration: %s", e.what());
            throw;
        }
    }
    
    void loadCameraCalibration(CameraConfig& cfg, const YAML::Node& intrinsic, const YAML::Node& extrinsic)
    {
        // Load intrinsic parameters
        auto cam_intrinsic = intrinsic[cfg.id];
        auto K = cam_intrinsic["camera_matrix"]["data"].as<std::vector<double>>();
        auto D = cam_intrinsic["distortion_coefficients"]["data"].as<std::vector<double>>();
        
        cfg.camera_matrix = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) {
            cfg.camera_matrix.at<double>(i / 3, i % 3) = K[i];
        }
        
        cfg.dist_coeffs = cv::Mat(1, D.size(), CV_64F);
        for (size_t i = 0; i < D.size(); ++i) {
            cfg.dist_coeffs.at<double>(0, i) = D[i];
        }
        
        // Load extrinsic parameters
        auto cam_extrinsic = extrinsic[cfg.id];
        auto T = cam_extrinsic["extrinsic_matrix"].as<std::vector<double>>();
        
        cfg.T_lidar_to_cam = Eigen::Matrix4d::Zero();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                cfg.T_lidar_to_cam(i, j) = T[i * 4 + j];
            }
        }
    }
    
    void setupPublishers()
    {
        // Main fused output publisher
        fused_pub_ = this->create_publisher<ModifiedFloat32MultiArray>(
            fused_output_topic_, 10);
        
        // Debug image publishers for each camera
        if (enable_debug_viz_) {
            for (const auto& cam : camera_configs_) {
                auto debug_pub = this->create_publisher<Image>(cam.debug_image_topic, 10);
                debug_image_pubs_.push_back(debug_pub);
            }
        }
    }
    
    void setupSubscribers()
    {
        // QoS settings
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 1;
        
        // LiDAR bounding boxes subscriber
        lidar_sub_ = std::make_shared<message_filters::Subscriber<BoundingBox3DArray>>(
            this, lidar_boxes_topic_, qos_profile);
        
        // Create subscribers for each camera
        for (const auto& cam : camera_configs_) {
            auto det_sub = std::make_shared<message_filters::Subscriber<DetectionArray>>(
                this, cam.detections_topic, qos_profile);
            detection_subs_.push_back(det_sub);
            
            if (enable_debug_viz_) {
                auto img_sub = std::make_shared<message_filters::Subscriber<Image>>(
                    this, cam.image_topic, qos_profile);
                image_subs_.push_back(img_sub);
            }
        }
        
        // Setup synchronizer for 2 cameras (most common case)
        if (camera_configs_.size() == 2 && enable_debug_viz_) {
            sync_2_ = std::make_shared<message_filters::Synchronizer<SyncPolicy2>>(
                SyncPolicy2(sync_queue_size_),
                *lidar_sub_,
                *detection_subs_[0], *detection_subs_[1],
                *image_subs_[0], *image_subs_[1]);
            
            sync_2_->registerCallback(
                std::bind(&MultiIoUFusionNode::syncCallback2WithImages, this,
                         std::placeholders::_1, std::placeholders::_2, 
                         std::placeholders::_3, std::placeholders::_4,
                         std::placeholders::_5));
                         
        } else if (camera_configs_.size() == 2) {
            // Without images for visualization
            using SyncPolicyNoImg = message_filters::sync_policies::ApproximateTime<
                BoundingBox3DArray, DetectionArray, DetectionArray>;
            auto sync_no_img = std::make_shared<message_filters::Synchronizer<SyncPolicyNoImg>>(
                SyncPolicyNoImg(sync_queue_size_),
                *lidar_sub_, *detection_subs_[0], *detection_subs_[1]);
            
            sync_no_img->registerCallback(
                std::bind(&MultiIoUFusionNode::syncCallback2NoImages, this,
                         std::placeholders::_1, std::placeholders::_2, 
                         std::placeholders::_3));
        }
        
        RCLCPP_INFO(this->get_logger(), "Synchronizer setup complete");
    }
    
    void syncCallback2WithImages(
        const BoundingBox3DArray::ConstSharedPtr& lidar_boxes,
        const DetectionArray::ConstSharedPtr& detections1,
        const DetectionArray::ConstSharedPtr& detections2,
        const Image::ConstSharedPtr& image1,
        const Image::ConstSharedPtr& image2)
    {
        std::vector<DetectionArray::ConstSharedPtr> detections = {detections1, detections2};
        std::vector<Image::ConstSharedPtr> images = {image1, image2};
        
        processFusion(lidar_boxes, detections, images);
    }
    
    void syncCallback2NoImages(
        const BoundingBox3DArray::ConstSharedPtr& lidar_boxes,
        const DetectionArray::ConstSharedPtr& detections1,
        const DetectionArray::ConstSharedPtr& detections2)
    {
        std::vector<DetectionArray::ConstSharedPtr> detections = {detections1, detections2};
        std::vector<Image::ConstSharedPtr> images;  // Empty
        
        processFusion(lidar_boxes, detections, images);
    }
    
    void processFusion(
        const BoundingBox3DArray::ConstSharedPtr& lidar_boxes,
        const std::vector<DetectionArray::ConstSharedPtr>& detections,
        const std::vector<Image::ConstSharedPtr>& images)
    {
        // Store fusion results
        std::vector<std::vector<std::string>> camera_class_names(camera_configs_.size());
        
        // Process each camera
        for (size_t cam_idx = 0; cam_idx < camera_configs_.size(); ++cam_idx) {
            const auto& cam_cfg = camera_configs_[cam_idx];
            const auto& yolo_detections = detections[cam_idx];
            
            // Project 3D boxes to this camera's 2D plane
            std::vector<cv::Rect2f> projected_boxes;
            for (const auto& box3d : lidar_boxes->boxes) {
                auto box2d = project3DBoxTo2D(box3d, cam_cfg);
                projected_boxes.push_back(box2d);
            }
            
            // Convert YOLO detections to 2D boxes
            std::vector<cv::Rect2f> yolo_boxes;
            for (const auto& det : yolo_detections->detections) {
                cv::Rect2f box(
                    det.bbox.center.position.x - det.bbox.size.x / 2,
                    det.bbox.center.position.y - det.bbox.size.y / 2,
                    det.bbox.size.x,
                    det.bbox.size.y);
                yolo_boxes.push_back(box);
            }
            
            // Compute IoU-based cost matrix
            Eigen::MatrixXd cost_matrix = computeIoUCostMatrix(projected_boxes, yolo_boxes);
            
            // Perform Hungarian matching
            auto match_result = matcher_->match(cost_matrix, 1.0 - iou_threshold_);
            
            // Store matched class names
            std::vector<std::string> class_names(lidar_boxes->boxes.size(), "Unknown");
            for (const auto& [yolo_idx, lidar_idx] : match_result.matches) {
                if (cost_matrix(yolo_idx, lidar_idx) < 1.0 - iou_threshold_) {
                    class_names[lidar_idx] = yolo_detections->detections[yolo_idx].class_name;
                }
            }
            camera_class_names[cam_idx] = class_names;
            
            // Visualize if enabled
            if (enable_debug_viz_ && !images.empty()) {
                visualizeMatching(images[cam_idx], projected_boxes, yolo_boxes, 
                                 match_result, cam_idx, cost_matrix);
            }
        }
        
        // Merge results from multiple cameras (voting or priority-based)
        std::vector<std::string> final_class_names = mergeClassifications(camera_class_names);
        
        // Publish fused results
        publishFusedResults(lidar_boxes, final_class_names);
    }
    
    cv::Rect2f project3DBoxTo2D(const vision_msgs::msg::BoundingBox3D& box3d, 
                                 const CameraConfig& cam_cfg)
    {
        // Get 8 corners of AABB (ignoring orientation for cones)
        std::vector<cv::Point3f> corners;
        double cx = box3d.center.position.x;
        double cy = box3d.center.position.y;
        double cz = box3d.center.position.z;
        double dx = box3d.size.x / 2.0;
        double dy = box3d.size.y / 2.0;
        double dz = box3d.size.z / 2.0;
        
        for (int i = 0; i < 8; ++i) {
            corners.emplace_back(
                cx + (i & 1 ? dx : -dx),
                cy + (i & 2 ? dy : -dy),
                cz + (i & 4 ? dz : -dz));
        }
        
        // Transform to camera frame and project
        std::vector<cv::Point2f> projected;
        for (const auto& pt : corners) {
            Eigen::Vector4d pt_lidar(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d pt_cam = cam_cfg.T_lidar_to_cam * pt_lidar;
            
            if (pt_cam(2) > 0.1) {  // In front of camera
                cv::Point3d pt3d(pt_cam(0), pt_cam(1), pt_cam(2));
                cv::Point2d pt2d;
                cv::projectPoints(std::vector<cv::Point3d>{pt3d}, 
                                 cv::Vec3d::zeros(), cv::Vec3d::zeros(),
                                 cam_cfg.camera_matrix, cam_cfg.dist_coeffs,
                                 std::vector<cv::Point2d>{pt2d});
                projected.push_back(cv::Point2f(pt2d.x, pt2d.y));
            }
        }
        
        // Find bounding rectangle
        if (projected.empty()) {
            return cv::Rect2f(0, 0, 0, 0);
        }
        
        return cv::boundingRect(projected);
    }
    
    Eigen::MatrixXd computeIoUCostMatrix(const std::vector<cv::Rect2f>& boxes1,
                                          const std::vector<cv::Rect2f>& boxes2)
    {
        Eigen::MatrixXd cost_matrix(boxes2.size(), boxes1.size());
        
        for (size_t i = 0; i < boxes2.size(); ++i) {
            for (size_t j = 0; j < boxes1.size(); ++j) {
                float iou = computeIoU(boxes2[i], boxes1[j]);
                cost_matrix(i, j) = 1.0 - iou;  // Cost = 1 - IoU
            }
        }
        
        return cost_matrix;
    }
    
    float computeIoU(const cv::Rect2f& box1, const cv::Rect2f& box2)
    {
        float x1 = std::max(box1.x, box2.x);
        float y1 = std::max(box1.y, box2.y);
        float x2 = std::min(box1.x + box1.width, box2.x + box2.width);
        float y2 = std::min(box1.y + box1.height, box2.y + box2.height);
        
        float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
        float area1 = box1.width * box1.height;
        float area2 = box2.width * box2.height;
        float union_area = area1 + area2 - intersection;
        
        return union_area > 0 ? intersection / union_area : 0.0f;
    }
    
    std::vector<std::string> mergeClassifications(
        const std::vector<std::vector<std::string>>& camera_classes)
    {
        if (camera_classes.empty()) return {};
        
        size_t num_cones = camera_classes[0].size();
        std::vector<std::string> merged(num_cones, "Unknown");
        
        // Simple voting or priority-based merging
        for (size_t i = 0; i < num_cones; ++i) {
            std::map<std::string, int> vote_count;
            for (const auto& cam_classes : camera_classes) {
                if (cam_classes[i] != "Unknown") {
                    vote_count[cam_classes[i]]++;
                }
            }
            
            // Select class with most votes
            int max_votes = 0;
            for (const auto& [class_name, votes] : vote_count) {
                if (votes > max_votes) {
                    max_votes = votes;
                    merged[i] = class_name;
                }
            }
        }
        
        return merged;
    }
    
    void publishFusedResults(const BoundingBox3DArray::ConstSharedPtr& lidar_boxes,
                            const std::vector<std::string>& class_names)
    {
        ModifiedFloat32MultiArray msg;
        msg.header = lidar_boxes->header;
        
        // Pack 3D positions
        for (const auto& box : lidar_boxes->boxes) {
            msg.data.push_back(box.center.position.x);
            msg.data.push_back(box.center.position.y);
            msg.data.push_back(box.center.position.z);
        }
        
        // Add class names
        msg.class_names = class_names;
        
        // Set layout
        if (!lidar_boxes->boxes.empty()) {
            std_msgs::msg::MultiArrayDimension dim1, dim2;
            dim1.label = "cones";
            dim1.size = lidar_boxes->boxes.size();
            dim1.stride = lidar_boxes->boxes.size() * 3;
            dim2.label = "coords";
            dim2.size = 3;
            dim2.stride = 3;
            msg.layout.dim.push_back(dim1);
            msg.layout.dim.push_back(dim2);
        }
        
        fused_pub_->publish(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published %zu fused cones", lidar_boxes->boxes.size());
    }
    
    void visualizeMatching(const Image::ConstSharedPtr& image,
                          const std::vector<cv::Rect2f>& projected_boxes,
                          const std::vector<cv::Rect2f>& yolo_boxes,
                          const fusion::MatchResult& matches,
                          size_t cam_idx,
                          const Eigen::MatrixXd& cost_matrix)
    {
        if (cam_idx >= debug_image_pubs_.size()) return;
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat& img = cv_ptr->image;
        
        // Draw projected LiDAR boxes (green)
        for (const auto& box : projected_boxes) {
            cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);
        }
        
        // Draw YOLO boxes (blue)
        for (const auto& box : yolo_boxes) {
            cv::rectangle(img, box, cv::Scalar(255, 0, 0), 2);
        }
        
        // Highlight matched pairs (yellow) with IoU scores
        for (const auto& [yolo_idx, lidar_idx] : matches.matches) {
            if (yolo_idx < yolo_boxes.size() && lidar_idx < projected_boxes.size()) {
                float iou = 1.0 - cost_matrix(yolo_idx, lidar_idx);
                cv::rectangle(img, projected_boxes[lidar_idx], cv::Scalar(0, 255, 255), 3);
                cv::rectangle(img, yolo_boxes[yolo_idx], cv::Scalar(0, 255, 255), 3);
                
                // Draw IoU score
                cv::Point text_pos(yolo_boxes[yolo_idx].x, yolo_boxes[yolo_idx].y - 5);
                std::string iou_text = "IoU: " + std::to_string(iou).substr(0, 4);
                cv::putText(img, iou_text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 
                           0.5, cv::Scalar(0, 255, 255), 2);
            }
        }
        
        // Publish debug image
        debug_image_pubs_[cam_idx]->publish(*cv_ptr->toImageMsg());
    }
};

} // namespace nodes
} // namespace calico

RCLCPP_COMPONENTS_REGISTER_NODE(calico::nodes::MultiIoUFusionNode)
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<calico::nodes::MultiIoUFusionNode>();
    
    RCLCPP_INFO(node->get_logger(), "Multi-Camera IoU Fusion Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
