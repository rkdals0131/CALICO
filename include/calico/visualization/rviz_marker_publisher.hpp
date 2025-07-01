#ifndef CALICO_VISUALIZATION_RVIZ_MARKER_PUBLISHER_HPP
#define CALICO_VISUALIZATION_RVIZ_MARKER_PUBLISHER_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/time.hpp>
#include "calico/utils/message_converter.hpp"

namespace calico {
namespace visualization {

/**
 * @brief Color definition for cone visualization
 */
struct ConeColor {
    float r, g, b, a;
    ConeColor(float r = 1.0f, float g = 1.0f, float b = 1.0f, float a = 1.0f)
        : r(r), g(g), b(b), a(a) {}
};

/**
 * @brief RViz marker publisher for cone visualization
 * 
 * Creates and publishes cone markers for visualization in RViz
 */
class RVizMarkerPublisher {
public:
    RVizMarkerPublisher();
    ~RVizMarkerPublisher() = default;
    
    /**
     * @brief Create marker array from tracked cones
     * @param cones Vector of tracked cones
     * @param frame_id Reference frame for markers
     * @param timestamp Marker timestamp
     * @return Marker array message
     */
    visualization_msgs::msg::MarkerArray createMarkerArray(
        const std::vector<utils::Cone>& cones,
        const std::string& frame_id = "ouster_lidar",
        const rclcpp::Time& timestamp = rclcpp::Time());
    
    /**
     * @brief Set cone visualization parameters
     * @param height Cone height in meters
     * @param radius Cone base radius in meters
     */
    void setConeParameters(double height, double radius);
    
    /**
     * @brief Set color for a specific cone type
     * @param color_name Cone color/type name
     * @param color RGB color values
     */
    void setColorMapping(const std::string& color_name, const ConeColor& color);
    
    /**
     * @brief Enable/disable track ID visualization
     * @param enable True to show track IDs
     */
    void setShowTrackIds(bool enable) { show_track_ids_ = enable; }
    
    /**
     * @brief Enable/disable cone color text labels
     * @param enable True to show color labels
     */
    void setShowColorLabels(bool enable) { show_color_labels_ = enable; }
    
private:
    /**
     * @brief Create a single cone marker
     * @param cone Cone data
     * @param marker_id Unique marker ID
     * @param frame_id Reference frame
     * @param timestamp Marker timestamp
     * @return Cone marker
     */
    visualization_msgs::msg::Marker createConeMarker(
        const utils::Cone& cone,
        int marker_id,
        const std::string& frame_id,
        const rclcpp::Time& timestamp);
    
    /**
     * @brief Create text marker for track ID
     * @param cone Cone data with track ID
     * @param marker_id Unique marker ID
     * @param frame_id Reference frame
     * @param timestamp Marker timestamp
     * @return Text marker
     */
    visualization_msgs::msg::Marker createTrackIdMarker(
        const utils::Cone& cone,
        int marker_id,
        const std::string& frame_id,
        const rclcpp::Time& timestamp);
    
    /**
     * @brief Create text marker for color label
     * @param cone Cone data with color
     * @param marker_id Unique marker ID
     * @param frame_id Reference frame
     * @param timestamp Marker timestamp
     * @return Text marker
     */
    visualization_msgs::msg::Marker createColorLabelMarker(
        const utils::Cone& cone,
        int marker_id,
        const std::string& frame_id,
        const rclcpp::Time& timestamp);
    
    /**
     * @brief Get color for cone type
     * @param color_name Cone color/type name
     * @return RGBA color
     */
    std_msgs::msg::ColorRGBA getColor(const std::string& color_name);
    
    /**
     * @brief Create delete marker to clear old markers
     * @param num_markers Number of markers to delete
     * @return Delete all marker
     */
    visualization_msgs::msg::Marker createDeleteAllMarker();
    
private:
    // Cone parameters
    double cone_height_;
    double cone_radius_;
    
    // Visualization options
    bool show_track_ids_;
    bool show_color_labels_;
    
    // Color mappings
    std::unordered_map<std::string, ConeColor> color_map_;
    
    // Marker tracking
    int last_marker_count_;
};

} // namespace visualization
} // namespace calico

#endif // CALICO_VISUALIZATION_RVIZ_MARKER_PUBLISHER_HPP