#ifndef CALICO_TRACKING_TRACK_HPP
#define CALICO_TRACKING_TRACK_HPP

#include <deque>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace calico {
namespace tracking {

/**
 * @brief Single object track using Unscented Kalman Filter
 * 
 * Represents a tracked cone with state estimation and color history
 */
class Track {
public:
    /**
     * @brief Constructor
     * @param track_id Unique track identifier
     * @param initial_x Initial X position
     * @param initial_y Initial Y position
     * @param initial_z Initial Z position (not filtered)
     */
    Track(int track_id, double initial_x, double initial_y, double initial_z);
    ~Track() = default;
    
    /**
     * @brief Predict next state using motion model
     * @param dt Time delta since last update
     * @param ax Optional X acceleration (from IMU)
     * @param ay Optional Y acceleration (from IMU)
     */
    void predict(double dt, double ax = 0.0, double ay = 0.0);
    
    /**
     * @brief Update state with new measurement
     * @param x Measured X position
     * @param y Measured Y position
     * @param z Measured Z position (passed through)
     * @param color Detected color/class
     */
    void update(double x, double y, double z, const std::string& color);
    
    /**
     * @brief Get current position estimate
     * @return 3D position (x, y, z)
     */
    Eigen::Vector3d getPosition() const;
    
    /**
     * @brief Get current velocity estimate
     * @return 2D velocity (vx, vy)
     */
    Eigen::Vector2d getVelocity() const;
    
    /**
     * @brief Get most likely color based on history
     * @return Color string with highest confidence
     */
    std::string getColor() const;
    
    /**
     * @brief Get track ID
     * @return Unique track identifier
     */
    int getId() const { return track_id_; }
    
    /**
     * @brief Get track age (frames since creation)
     * @return Track age
     */
    int getAge() const { return age_; }
    
    /**
     * @brief Get number of successful updates
     * @return Hit count
     */
    int getHitCount() const { return hit_count_; }
    
    /**
     * @brief Get frames since last update
     * @return Time since last hit
     */
    int getTimeSinceUpdate() const { return time_since_update_; }
    
    /**
     * @brief Check if track is confirmed (enough hits)
     * @param min_hits Minimum hits required
     * @return True if confirmed
     */
    bool isConfirmed(int min_hits = 3) const { return hit_count_ >= min_hits; }
    
    /**
     * @brief Increment time since update (called when no match)
     */
    void incrementTimeSinceUpdate() { time_since_update_++; }
    
private:
    /**
     * @brief Initialize UKF parameters
     * @param x Initial X position
     * @param y Initial Y position
     */
    void initializeUKF(double x, double y);
    
    /**
     * @brief Generate sigma points for UKF
     * @return Matrix of sigma points
     */
    Eigen::MatrixXd generateSigmaPoints();
    
    /**
     * @brief Predict sigma points through motion model
     * @param sigma_points Input sigma points
     * @param dt Time delta
     * @param ax X acceleration
     * @param ay Y acceleration
     * @return Predicted sigma points
     */
    Eigen::MatrixXd predictSigmaPoints(const Eigen::MatrixXd& sigma_points,
                                      double dt, double ax, double ay);
    
    /**
     * @brief Compute weights for sigma points
     */
    void computeWeights();
    
    /**
     * @brief Update color history and vote
     * @param color New color observation
     */
    void updateColorHistory(const std::string& color);
    
private:
    // Track identification
    int track_id_;
    int age_;
    int hit_count_;
    int time_since_update_;
    
    // UKF state: [x, y, vx, vy]
    Eigen::Vector4d state_;
    Eigen::Matrix4d covariance_;
    
    // UKF parameters
    static constexpr int STATE_DIM = 4;
    static constexpr int MEAS_DIM = 2;
    double alpha_, beta_, kappa_, lambda_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;
    
    // Process and measurement noise
    Eigen::Matrix4d Q_;  // Process noise covariance
    Eigen::Matrix2d R_;  // Measurement noise covariance
    
    // Z coordinate (not filtered)
    double z_;
    
    // Color history for voting
    std::deque<std::string> color_history_;
    static constexpr size_t MAX_COLOR_HISTORY = 20;
};

} // namespace tracking
} // namespace calico

#endif // CALICO_TRACKING_TRACK_HPP