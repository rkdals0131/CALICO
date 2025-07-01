#include <rclcpp/rclcpp.hpp>
#include <custom_interface/msg/modified_float32_multi_array.hpp>
#include <custom_interface/msg/tracked_cone_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "calico/tracking/ukf_tracker.hpp"
#include "calico/utils/message_converter.hpp"

namespace calico {

class UKFTrackingNode : public rclcpp::Node {
public:
    UKFTrackingNode() : Node("calico_ukf_tracking") {
        RCLCPP_INFO(this->get_logger(), "Initializing CALICO UKF Tracking Node");
        
        // Declare parameters (matching Python defaults)
        this->declare_parameter<double>("q_pos", 0.1);
        this->declare_parameter<double>("q_vel", 0.1);
        this->declare_parameter<double>("r_pos", 0.1);  // Python default
        this->declare_parameter<double>("alpha", 0.001);
        this->declare_parameter<double>("beta", 2.0);
        this->declare_parameter<double>("kappa", 0.0);
        this->declare_parameter<int>("max_age_before_deletion", 4);  // Python default
        this->declare_parameter<int>("min_hits_before_confirmation", 3);
        this->declare_parameter<double>("max_association_distance", 0.7);  // Python default
        this->declare_parameter<bool>("use_imu", true);
        
        // Load parameters
        tracking::UKFConfig config;
        config.q_pos = this->get_parameter("q_pos").as_double();
        config.q_vel = this->get_parameter("q_vel").as_double();
        config.r_pos = this->get_parameter("r_pos").as_double();
        config.alpha = this->get_parameter("alpha").as_double();
        config.beta = this->get_parameter("beta").as_double();
        config.kappa = this->get_parameter("kappa").as_double();
        config.max_age_before_deletion = this->get_parameter("max_age_before_deletion").as_int();
        config.min_hits_before_confirmation = this->get_parameter("min_hits_before_confirmation").as_int();
        config.max_association_distance = this->get_parameter("max_association_distance").as_double();
        
        use_imu_ = this->get_parameter("use_imu").as_bool();
        
        // Initialize tracker
        tracker_ = std::make_unique<tracking::UKFTracker>(config);
        
        // Setup QoS
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // Create subscribers
        cones_sub_ = this->create_subscription<custom_interface::msg::ModifiedFloat32MultiArray>(
            "/fused_sorted_cones", qos,
            std::bind(&UKFTrackingNode::conesCallback, this, std::placeholders::_1));
        
        if (use_imu_) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/ouster/imu", qos,
                std::bind(&UKFTrackingNode::imuCallback, this, std::placeholders::_1));
        }
        
        // Create publisher
        tracked_cones_pub_ = this->create_publisher<custom_interface::msg::TrackedConeArray>(
            "/fused_sorted_cones_ukf", qos);
        
        RCLCPP_INFO(this->get_logger(), "UKF Tracking Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "IMU usage: %s", use_imu_ ? "enabled" : "disabled");
    }
    
private:
    void conesCallback(const custom_interface::msg::ModifiedFloat32MultiArray::SharedPtr msg) {
        // Convert message to internal representation
        auto cones = utils::MessageConverter::fromModifiedFloat32MultiArray(*msg);
        
        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // Update tracker
        if (use_imu_ && latest_imu_) {
            auto imu_data = utils::MessageConverter::fromImuMsg(*latest_imu_);
            tracker_->update(cones, timestamp, &imu_data);
        } else {
            tracker_->update(cones, timestamp, nullptr);
        }
        
        // Get tracked cones
        auto tracked_cones = tracker_->getTrackedCones();
        
        // Convert to ROS message
        auto output_msg = utils::MessageConverter::toTrackedConeArray(tracked_cones);
        output_msg.header = msg->header;
        
        // Publish
        tracked_cones_pub_->publish(output_msg);
        
        // Log statistics
        static int update_count = 0;
        if (++update_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Tracking %zu cones, %zu active tracks",
                       tracked_cones.size(), tracker_->getTracks().size());
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        latest_imu_ = msg;
    }
    
private:
    // Tracker
    std::unique_ptr<tracking::UKFTracker> tracker_;
    
    // Subscribers
    rclcpp::Subscription<custom_interface::msg::ModifiedFloat32MultiArray>::SharedPtr cones_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // Publisher
    rclcpp::Publisher<custom_interface::msg::TrackedConeArray>::SharedPtr tracked_cones_pub_;
    
    // Latest IMU data
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    bool use_imu_;
};

} // namespace calico

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<calico::UKFTrackingNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}