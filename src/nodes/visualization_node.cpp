#include <rclcpp/rclcpp.hpp>
#include <custom_interface/msg/tracked_cone_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "calico/visualization/rviz_marker_publisher.hpp"
#include "calico/utils/message_converter.hpp"

namespace calico {

class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("calico_visualization") {
        RCLCPP_INFO(this->get_logger(), "Initializing CALICO Visualization Node");
        
        // Declare parameters
        this->declare_parameter<double>("cone_height", 0.5);
        this->declare_parameter<double>("cone_radius", 0.15);
        this->declare_parameter<bool>("show_track_ids", true);
        this->declare_parameter<bool>("show_color_labels", false);
        this->declare_parameter<std::string>("frame_id", "ouster_lidar");
        
        // Load parameters
        double cone_height = this->get_parameter("cone_height").as_double();
        double cone_radius = this->get_parameter("cone_radius").as_double();
        show_track_ids_ = this->get_parameter("show_track_ids").as_bool();
        show_color_labels_ = this->get_parameter("show_color_labels").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Initialize marker publisher
        marker_publisher_ = std::make_unique<visualization::RVizMarkerPublisher>();
        marker_publisher_->setConeParameters(cone_height, cone_radius);
        marker_publisher_->setShowTrackIds(show_track_ids_);
        marker_publisher_->setShowColorLabels(show_color_labels_);
        
        // Setup QoS
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // Create subscriber
        tracked_cones_sub_ = this->create_subscription<custom_interface::msg::TrackedConeArray>(
            "/fused_sorted_cones_ukf", qos,
            std::bind(&VisualizationNode::trackedConesCallback, this, std::placeholders::_1));
        
        // Create publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_array", qos);
        
        RCLCPP_INFO(this->get_logger(), "Visualization Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Show track IDs: %s", show_track_ids_ ? "yes" : "no");
        RCLCPP_INFO(this->get_logger(), "Show color labels: %s", show_color_labels_ ? "yes" : "no");
    }
    
private:
    void trackedConesCallback(const custom_interface::msg::TrackedConeArray::SharedPtr msg) {
        // Convert to internal representation
        auto cones = utils::MessageConverter::fromTrackedConeArray(*msg);
        
        // Use frame_id from message if available, otherwise use parameter
        std::string frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
        
        // Create marker array
        auto marker_array = marker_publisher_->createMarkerArray(
            cones, frame_id, msg->header.stamp);
        
        // Publish markers
        marker_pub_->publish(marker_array);
        
        // Log statistics
        static int vis_count = 0;
        if (++vis_count % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "Visualized %zu cones (%d cycles)",
                        cones.size(), vis_count);
        }
    }
    
private:
    // Marker publisher
    std::unique_ptr<visualization::RVizMarkerPublisher> marker_publisher_;
    
    // Subscriber
    rclcpp::Subscription<custom_interface::msg::TrackedConeArray>::SharedPtr tracked_cones_sub_;
    
    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Parameters
    bool show_track_ids_;
    bool show_color_labels_;
    std::string frame_id_;
};

} // namespace calico

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<calico::VisualizationNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}