#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <custom_interface/msg/modified_float32_multi_array.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "calico/utils/config_loader.hpp"
#include "calico/utils/message_converter.hpp"
#include "calico/fusion/multi_camera_fusion.hpp"

namespace calico {

class MultiCameraFusionNode : public rclcpp::Node {
public:
    MultiCameraFusionNode() : Node("calico_multi_camera_fusion") {
        RCLCPP_INFO(this->get_logger(), "Initializing CALICO Multi-Camera Fusion Node");
        
        // Declare parameters
        this->declare_parameter<std::string>("config_file", "");
        
        // Load configuration
        std::string config_file = this->get_parameter("config_file").as_string();
        if (config_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No config file specified! Use --ros-args -p config_file:=path/to/config.yaml");
            throw std::runtime_error("Config file not specified");
        }
        
        try {
            utils::ConfigLoader loader;
            config_ = loader.loadConfig(config_file);
            RCLCPP_INFO(this->get_logger(), "Loaded configuration from: %s", config_file.c_str());
            RCLCPP_INFO(this->get_logger(), "Number of cameras: %zu", config_.cameras.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            throw;
        }
        
        // Initialize fusion module
        fusion_ = std::make_unique<fusion::MultiCameraFusion>();
        fusion_->initialize(config_.cameras);
        fusion_->setMaxMatchingDistance(config_.max_matching_distance);
        
        // Setup QoS
        rclcpp::QoS qos(config_.history_depth);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // Create publisher
        fused_cones_pub_ = this->create_publisher<custom_interface::msg::ModifiedFloat32MultiArray>(
            config_.output_topic, qos);
        
        // Create subscribers
        cones_sub_ = this->create_subscription<custom_interface::msg::ModifiedFloat32MultiArray>(
            config_.cones_topic, qos,
            std::bind(&MultiCameraFusionNode::conesCallback, this, std::placeholders::_1));
        
        // Create YOLO detection subscribers for each camera
        for (const auto& cam : config_.cameras) {
            auto sub = this->create_subscription<yolo_msgs::msg::DetectionArray>(
                cam.detections_topic, qos,
                [this, cam_id = cam.id](const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
                    this->detectionCallback(cam_id, msg);
                });
            detection_subs_[cam.id] = sub;
            
            RCLCPP_INFO(this->get_logger(), "Subscribed to camera %s on topic: %s", 
                       cam.id.c_str(), cam.detections_topic.c_str());
        }
        
        // Initialize buffers
        latest_cones_timestamp_ = rclcpp::Time(0);
        
        RCLCPP_INFO(this->get_logger(), "CALICO Multi-Camera Fusion Node initialized successfully");
        RCLCPP_WARN(this->get_logger(), "Note: This is a placeholder implementation. Full fusion logic pending.");
    }

private:
    void conesCallback(const custom_interface::msg::ModifiedFloat32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_cones_ = msg;
        latest_cones_timestamp_ = this->now();
        
        // Try to process if we have recent detections
        tryFusion();
    }
    
    void detectionCallback(const std::string& camera_id, 
                          const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_detections_[camera_id] = msg;
        latest_detection_timestamps_[camera_id] = this->now();
        
        // Try to process if we have recent cones
        tryFusion();
    }
    
    void tryFusion() {
        // Check if we have recent cone data
        if (!latest_cones_ || 
            (this->now() - latest_cones_timestamp_).seconds() > config_.sync_slop) {
            return;
        }
        
        // Check if we have recent detections from at least one camera
        bool has_recent_detection = false;
        for (const auto& cam : config_.cameras) {
            auto it = latest_detection_timestamps_.find(cam.id);
            if (it != latest_detection_timestamps_.end() &&
                (this->now() - it->second).seconds() <= config_.sync_slop) {
                has_recent_detection = true;
                break;
            }
        }
        
        if (!has_recent_detection) {
            return;
        }
        
        // Perform fusion
        performFusion();
    }
    
    void performFusion() {
        RCLCPP_DEBUG(this->get_logger(), "Performing fusion...");
        
        // Convert messages to internal representation
        auto cones = utils::MessageConverter::fromModifiedFloat32MultiArray(*latest_cones_);
        
        // Convert YOLO detections for each camera
        std::unordered_map<std::string, std::vector<utils::Detection>> camera_detections;
        for (const auto& [cam_id, det_msg] : latest_detections_) {
            if (det_msg) {
                camera_detections[cam_id] = utils::MessageConverter::fromDetectionArray(*det_msg);
            }
        }
        
        // Perform fusion
        auto fused_cones = fusion_->fuse(cones, camera_detections);
        
        // Convert result back to ROS message
        auto output_msg = utils::MessageConverter::toModifiedFloat32MultiArray(fused_cones);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = latest_cones_->header.frame_id;
        
        fused_cones_pub_->publish(output_msg);
        
        // Log some statistics
        static int fusion_count = 0;
        if (++fusion_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Processed %d fusion cycles", fusion_count);
            
            // Log fusion results
            int matched_cones = 0;
            for (const auto& cone : fused_cones) {
                if (cone.color != "Unknown") {
                    matched_cones++;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Matched %d/%zu cones with YOLO detections", 
                       matched_cones, fused_cones.size());
            
            // Debug camera detection info
            for (const auto& [cam_id, det_msg] : latest_detections_) {
                if (det_msg) {
                    RCLCPP_INFO(this->get_logger(), "Camera %s: %zu YOLO detections", 
                               cam_id.c_str(), det_msg->detections.size());
                }
            }
            
            // Debug fusion results per camera
            auto& cam_results = fusion_->getCameraResults();
            for (const auto& [cam_id, result] : cam_results) {
                int matched = 0;
                for (int idx : result.matched_cone_indices) {
                    if (idx >= 0) matched++;
                }
                RCLCPP_INFO(this->get_logger(), "Camera %s fusion: %d matched, %zu unmatched",
                           cam_id.c_str(), matched, result.unmatched_cone_indices.size());
            }
        }
    }

private:
    // Configuration
    utils::CalicoConfig config_;
    
    // Fusion module
    std::unique_ptr<fusion::MultiCameraFusion> fusion_;
    
    // Publishers
    rclcpp::Publisher<custom_interface::msg::ModifiedFloat32MultiArray>::SharedPtr fused_cones_pub_;
    
    // Subscribers
    rclcpp::Subscription<custom_interface::msg::ModifiedFloat32MultiArray>::SharedPtr cones_sub_;
    std::unordered_map<std::string, 
        rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr> detection_subs_;
    
    // Data buffers
    std::mutex data_mutex_;
    custom_interface::msg::ModifiedFloat32MultiArray::SharedPtr latest_cones_;
    rclcpp::Time latest_cones_timestamp_;
    std::unordered_map<std::string, yolo_msgs::msg::DetectionArray::SharedPtr> latest_detections_;
    std::unordered_map<std::string, rclcpp::Time> latest_detection_timestamps_;
};

} // namespace calico

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<calico::MultiCameraFusionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}