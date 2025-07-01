#include "calico/visualization/rviz_marker_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

namespace calico {
namespace visualization {

RVizMarkerPublisher::RVizMarkerPublisher()
    : cone_height_(0.5),
      cone_radius_(0.15),
      show_track_ids_(true),
      show_color_labels_(false),
      last_marker_count_(0) {
    
    // Initialize default color mappings (matching Python lowercase format)
    color_map_["blue cone"] = ConeColor(0.0f, 0.0f, 1.0f, 1.0f);
    color_map_["yellow cone"] = ConeColor(1.0f, 1.0f, 0.0f, 1.0f);
    color_map_["orange cone"] = ConeColor(1.0f, 0.5f, 0.0f, 1.0f);
    color_map_["red cone"] = ConeColor(1.0f, 0.0f, 0.0f, 1.0f);
    color_map_["unknown"] = ConeColor(0.5f, 0.5f, 0.5f, 0.8f);
    // Also support capitalized versions
    color_map_["Blue"] = ConeColor(0.0f, 0.0f, 1.0f, 1.0f);
    color_map_["Yellow"] = ConeColor(1.0f, 1.0f, 0.0f, 1.0f);
    color_map_["Orange"] = ConeColor(1.0f, 0.5f, 0.0f, 1.0f);
    color_map_["Red"] = ConeColor(1.0f, 0.0f, 0.0f, 1.0f);
    color_map_["Unknown"] = ConeColor(0.5f, 0.5f, 0.5f, 0.8f);
}

visualization_msgs::msg::MarkerArray RVizMarkerPublisher::createMarkerArray(
    const std::vector<utils::Cone>& cones,
    const std::string& frame_id,
    const rclcpp::Time& timestamp) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    
    // Add delete all marker to clear old markers
    if (last_marker_count_ > cones.size() * 3) {
        marker_array.markers.push_back(createDeleteAllMarker());
        marker_id++;
    }
    
    // Create cone markers
    for (const auto& cone : cones) {
        // Cone mesh marker
        marker_array.markers.push_back(
            createConeMarker(cone, marker_id++, frame_id, timestamp));
        
        // Track ID text
        if (show_track_ids_ && cone.id >= 0) {
            marker_array.markers.push_back(
                createTrackIdMarker(cone, marker_id++, frame_id, timestamp));
        }
        
        // Color label text
        if (show_color_labels_) {
            marker_array.markers.push_back(
                createColorLabelMarker(cone, marker_id++, frame_id, timestamp));
        }
    }
    
    last_marker_count_ = marker_id;
    return marker_array;
}

void RVizMarkerPublisher::setConeParameters(double height, double radius) {
    cone_height_ = height;
    cone_radius_ = radius;
}

void RVizMarkerPublisher::setColorMapping(const std::string& color_name, const ConeColor& color) {
    color_map_[color_name] = color;
}

visualization_msgs::msg::Marker RVizMarkerPublisher::createConeMarker(
    const utils::Cone& cone,
    int marker_id,
    const std::string& frame_id,
    const rclcpp::Time& timestamp) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "cones";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position
    marker.pose.position.x = cone.x;
    marker.pose.position.y = cone.y;
    marker.pose.position.z = cone.z + cone_height_ / 2.0;  // Center of cylinder
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Scale
    marker.scale.x = cone_radius_ * 2.0;
    marker.scale.y = cone_radius_ * 2.0;
    marker.scale.z = cone_height_;
    
    // Color
    marker.color = getColor(cone.color);
    
    // Lifetime
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    return marker;
}

visualization_msgs::msg::Marker RVizMarkerPublisher::createTrackIdMarker(
    const utils::Cone& cone,
    int marker_id,
    const std::string& frame_id,
    const rclcpp::Time& timestamp) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "track_ids";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position (above cone)
    marker.pose.position.x = cone.x;
    marker.pose.position.y = cone.y;
    marker.pose.position.z = cone.z + cone_height_ + 0.2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Text
    marker.text = std::to_string(cone.id);
    
    // Scale
    marker.scale.z = 0.2;  // Text height
    
    // Color (white)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    // Lifetime
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    return marker;
}

visualization_msgs::msg::Marker RVizMarkerPublisher::createColorLabelMarker(
    const utils::Cone& cone,
    int marker_id,
    const std::string& frame_id,
    const rclcpp::Time& timestamp) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "color_labels";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position (to the side of cone)
    marker.pose.position.x = cone.x + 0.3;
    marker.pose.position.y = cone.y;
    marker.pose.position.z = cone.z + cone_height_ / 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Text
    marker.text = cone.color;
    
    // Scale
    marker.scale.z = 0.15;  // Text height
    
    // Color (match cone color)
    marker.color = getColor(cone.color);
    
    // Lifetime
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    return marker;
}

std_msgs::msg::ColorRGBA RVizMarkerPublisher::getColor(const std::string& color_name) {
    std_msgs::msg::ColorRGBA color;
    
    auto it = color_map_.find(color_name);
    if (it != color_map_.end()) {
        color.r = it->second.r;
        color.g = it->second.g;
        color.b = it->second.b;
        color.a = it->second.a;
    } else {
        // Default to gray for unknown colors
        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.5;
        color.a = 0.8;
    }
    
    return color;
}

visualization_msgs::msg::Marker RVizMarkerPublisher::createDeleteAllMarker() {
    visualization_msgs::msg::Marker marker;
    marker.ns = "cones";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
}

} // namespace visualization
} // namespace calico