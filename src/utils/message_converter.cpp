#include "calico/utils/message_converter.hpp"
#include <rclcpp/rclcpp.hpp>

namespace calico {
namespace utils {

std::vector<Cone> MessageConverter::fromModifiedFloat32MultiArray(
    const custom_interface::msg::ModifiedFloat32MultiArray& msg) {
    std::vector<Cone> cones;
    
    // Check if data is valid
    if (msg.data.empty() || msg.data.size() % 3 != 0) {
        RCLCPP_WARN(rclcpp::get_logger("message_converter"), 
                   "Invalid cone data size: %zu", msg.data.size());
        return cones;
    }
    
    // Extract cones from flat array (x, y, z triplets)
    size_t num_cones = msg.data.size() / 3;
    size_t num_classes = msg.class_names.size();
    
    for (size_t i = 0; i < num_cones; ++i) {
        Cone cone;
        cone.x = msg.data[i * 3];
        cone.y = msg.data[i * 3 + 1];
        cone.z = msg.data[i * 3 + 2];
        
        // Assign class name if available
        if (i < num_classes) {
            cone.color = msg.class_names[i];
        } else {
            cone.color = "Unknown";
        }
        
        cones.push_back(cone);
    }
    
    return cones;
}

custom_interface::msg::ModifiedFloat32MultiArray 
MessageConverter::toModifiedFloat32MultiArray(const std::vector<Cone>& cones) {
    custom_interface::msg::ModifiedFloat32MultiArray msg;
    
    // Reserve space
    msg.data.reserve(cones.size() * 3);
    msg.class_names.reserve(cones.size());
    
    // Populate data
    for (const auto& cone : cones) {
        msg.data.push_back(cone.x);
        msg.data.push_back(cone.y);
        msg.data.push_back(cone.z);
        msg.class_names.push_back(cone.color);
    }
    
    // Set layout information
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "cones";
    msg.layout.dim[0].size = cones.size();
    msg.layout.dim[0].stride = cones.size() * 3;
    msg.layout.dim[1].label = "xyz";
    msg.layout.dim[1].size = 3;
    msg.layout.dim[1].stride = 3;
    
    return msg;
}

std::vector<Detection> MessageConverter::fromDetectionArray(
    const yolo_msgs::msg::DetectionArray& msg) {
    std::vector<Detection> detections;
    
    for (const auto& det : msg.detections) {
        Detection detection;
        
        // Extract bounding box center and dimensions
        detection.x = det.bbox.center.position.x;
        detection.y = det.bbox.center.position.y;
        detection.width = det.bbox.size.x;
        detection.height = det.bbox.size.y;
        
        // Extract class and confidence
        detection.class_name = det.class_name;
        detection.confidence = det.score;
        
        detections.push_back(detection);
    }
    
    return detections;
}

std::vector<Cone> MessageConverter::fromTrackedConeArray(
    const custom_interface::msg::TrackedConeArray& msg) {
    std::vector<Cone> cones;
    
    for (const auto& tracked_cone : msg.cones) {
        Cone cone;
        cone.x = tracked_cone.position.x;
        cone.y = tracked_cone.position.y;
        cone.z = tracked_cone.position.z;
        cone.color = tracked_cone.color;
        cone.id = tracked_cone.track_id;
        cones.push_back(cone);
    }
    
    return cones;
}

custom_interface::msg::TrackedConeArray 
MessageConverter::toTrackedConeArray(const std::vector<Cone>& cones) {
    custom_interface::msg::TrackedConeArray msg;
    
    for (const auto& cone : cones) {
        custom_interface::msg::TrackedCone tracked_cone;
        tracked_cone.position.x = cone.x;
        tracked_cone.position.y = cone.y;
        tracked_cone.position.z = cone.z;
        tracked_cone.color = cone.color;
        tracked_cone.track_id = cone.id;
        msg.cones.push_back(tracked_cone);
    }
    
    return msg;
}

IMUData MessageConverter::fromImuMsg(const sensor_msgs::msg::Imu& msg) {
    IMUData imu_data;
    
    imu_data.linear_accel_x = msg.linear_acceleration.x;
    imu_data.linear_accel_y = msg.linear_acceleration.y;
    imu_data.linear_accel_z = msg.linear_acceleration.z;
    
    imu_data.angular_vel_x = msg.angular_velocity.x;
    imu_data.angular_vel_y = msg.angular_velocity.y;
    imu_data.angular_vel_z = msg.angular_velocity.z;
    
    imu_data.orientation = msg.orientation;
    imu_data.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    
    return imu_data;
}

std::pair<double, double> MessageConverter::getDetectionCenter(const Detection& detection) {
    return std::make_pair(detection.x, detection.y);
}

std::string MessageConverter::mapClassToColor(const std::string& class_name) {
    // Map YOLO class names to cone colors (matching Python)
    // Python uses lowercase color names: "blue cone", "red cone", "yellow cone"
    static const std::unordered_map<std::string, std::string> class_to_color = {
        // YOLO sends class names with spaces (e.g., "blue cone")
        {"blue cone", "blue cone"},
        {"yellow cone", "yellow cone"},
        {"orange cone", "orange cone"},
        {"red cone", "red cone"},
        // Also support underscore versions
        {"Blue_Cone", "blue cone"},
        {"Yellow_Cone", "yellow cone"},
        {"Orange_Cone", "orange cone"},
        {"Red_Cone", "red cone"},
        {"blue_cone", "blue cone"},
        {"yellow_cone", "yellow cone"},
        {"orange_cone", "orange cone"},
        {"red_cone", "red cone"}
    };
    
    auto it = class_to_color.find(class_name);
    if (it != class_to_color.end()) {
        return it->second;
    }
    
    return "Unknown";  // Python uses "Unknown" (capitalized)
}

// Convert Cone to kalman_filters Detection
kalman_filters::tracking::Detection MessageConverter::coneToDetection(const Cone& cone) {
    kalman_filters::tracking::Detection det;
    det.x = cone.x;
    det.y = cone.y;
    det.z = cone.z;
    det.label = cone.color;
    det.id = cone.id;
    det.confidence = cone.confidence;
    return det;
}

// Convert Detection to Cone
Cone MessageConverter::detectionToCone(const kalman_filters::tracking::Detection& detection) {
    Cone cone;
    cone.x = detection.x;
    cone.y = detection.y;
    cone.z = detection.z;
    cone.color = detection.label;
    cone.id = detection.id;
    cone.confidence = detection.confidence;
    return cone;
}

// Convert vector of Cones to Detections
std::vector<kalman_filters::tracking::Detection> MessageConverter::conesToDetections(const std::vector<Cone>& cones) {
    std::vector<kalman_filters::tracking::Detection> detections;
    detections.reserve(cones.size());
    for (const auto& cone : cones) {
        detections.push_back(coneToDetection(cone));
    }
    return detections;
}

// Convert vector of Detections to Cones
std::vector<Cone> MessageConverter::detectionsToCones(const std::vector<kalman_filters::tracking::Detection>& detections) {
    std::vector<Cone> cones;
    cones.reserve(detections.size());
    for (const auto& det : detections) {
        cones.push_back(detectionToCone(det));
    }
    return cones;
}

// Convert TrackedObject to Cone with velocity
Cone MessageConverter::trackedObjectToCone(const kalman_filters::tracking::TrackedObject& obj) {
    Cone cone;
    cone.x = obj.x;
    cone.y = obj.y;
    cone.z = obj.z;
    cone.color = obj.label;
    cone.id = obj.track_id;
    cone.confidence = obj.confidence;
    cone.has_velocity = true;
    cone.velocity = Eigen::Vector3d(obj.vx, obj.vy, 0.0);
    return cone;
}

// Convert vector of TrackedObjects to Cones
std::vector<Cone> MessageConverter::trackedObjectsToCones(const std::vector<kalman_filters::tracking::TrackedObject>& objects) {
    std::vector<Cone> cones;
    cones.reserve(objects.size());
    for (const auto& obj : objects) {
        cones.push_back(trackedObjectToCone(obj));
    }
    return cones;
}

// Convert IMUData to kalman_filters format
kalman_filters::tracking::IMUData MessageConverter::toKalmanIMUData(const IMUData& imu_data) {
    kalman_filters::tracking::IMUData kf_imu;
    kf_imu.timestamp = imu_data.timestamp;
    kf_imu.angular_vel_x = imu_data.angular_vel_x;
    kf_imu.angular_vel_y = imu_data.angular_vel_y;
    kf_imu.angular_vel_z = imu_data.angular_vel_z;
    kf_imu.linear_accel_x = imu_data.linear_accel_x;
    kf_imu.linear_accel_y = imu_data.linear_accel_y;
    kf_imu.linear_accel_z = imu_data.linear_accel_z;
    return kf_imu;
}

// Convert kalman_filters IMUData to calico format
IMUData MessageConverter::fromKalmanIMUData(const kalman_filters::tracking::IMUData& kf_imu_data) {
    IMUData imu;
    imu.timestamp = kf_imu_data.timestamp;
    imu.angular_vel_x = kf_imu_data.angular_vel_x;
    imu.angular_vel_y = kf_imu_data.angular_vel_y;
    imu.angular_vel_z = kf_imu_data.angular_vel_z;
    imu.linear_accel_x = kf_imu_data.linear_accel_x;
    imu.linear_accel_y = kf_imu_data.linear_accel_y;
    imu.linear_accel_z = kf_imu_data.linear_accel_z;
    // orientation은 칼만 필터에서 사용하지 않으므로 변환하지 않음
    return imu;
}

} // namespace utils
} // namespace calico