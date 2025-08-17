#ifndef CALICO_NODES_IOU_FUSION_NODE_HPP
#define CALICO_NODES_IOU_FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <custom_interface/msg/modified_float32_multi_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "calico/utils/projection_utils.hpp"
#include "calico/fusion/hungarian_matcher.hpp"

namespace calico {
namespace nodes {

/**
 * @brief IoU-based sensor fusion node for LiDAR and YOLO detections
 * 
 * This node performs sensor fusion using Intersection over Union (IoU) matching
 * between projected 3D LiDAR bounding boxes and 2D YOLO detections.
 */
class IoUFusionNode : public rclcpp::Node {
public:
    explicit IoUFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Callback functions
    void fusionCallback(
        const vision_msgs::msg::BoundingBox3DArray::SharedPtr lidar_msg,
        const yolo_msgs::msg::DetectionArray::SharedPtr yolo_msg,
        const sensor_msgs::msg::Image::SharedPtr image_msg);

    // Camera calibration loading
    void loadCameraCalibration();
    
    // 3D to 2D projection functions
    std::vector<cv::Point3f> get3DBoundingBoxCorners(
        const vision_msgs::msg::BoundingBox3D& bbox);
    
    cv::Rect2f project3DBoxTo2D(const std::vector<cv::Point3f>& corners_3d);
    
    // IoU calculation
    double calculateIoU(const cv::Rect2f& boxA, const cv::Rect2f& boxB);
    
    // Hungarian algorithm matching
    std::vector<std::pair<int, int>> performHungarianMatching(
        const std::vector<cv::Rect2f>& projected_boxes,
        const std::vector<cv::Rect2f>& yolo_boxes);
    
    // Visualization functions
    cv::Mat createDebugVisualization(
        const cv::Mat& image,
        const std::vector<cv::Rect2f>& projected_boxes,
        const std::vector<cv::Rect2f>& yolo_boxes,
        const std::vector<std::pair<int, int>>& matches,
        const std::vector<double>& iou_scores);
    
    // Message creation
    custom_interface::msg::ModifiedFloat32MultiArray createFusedMessage(
        const vision_msgs::msg::BoundingBox3DArray::SharedPtr& lidar_msg,
        const yolo_msgs::msg::DetectionArray::SharedPtr& yolo_msg,
        const std::vector<std::pair<int, int>>& matches);

    // Publishers and subscribers
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::BoundingBox3DArray>> lidar_sub_;
    std::shared_ptr<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>> yolo_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    
    rclcpp::Publisher<custom_interface::msg::ModifiedFloat32MultiArray>::SharedPtr fused_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    
    // Message synchronization
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        vision_msgs::msg::BoundingBox3DArray,
        yolo_msgs::msg::DetectionArray,
        sensor_msgs::msg::Image>;
    
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Camera calibration data
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    Eigen::Matrix4d T_lidar_to_cam_;
    
    // Configuration parameters
    double iou_threshold_;
    std::string config_file_path_;
    std::string camera_intrinsic_file_;
    std::string camera_extrinsic_file_;
    
    // Hungarian matcher instance
    std::unique_ptr<fusion::HungarianMatcher> hungarian_matcher_;
    
    // Visualization parameters
    cv::Scalar lidar_box_color_;
    cv::Scalar yolo_box_color_;
    cv::Scalar matched_box_color_;
    int line_thickness_;
    int font_face_;
    double font_scale_;
};

} // namespace nodes
} // namespace calico

#endif // CALICO_NODES_IOU_FUSION_NODE_HPP