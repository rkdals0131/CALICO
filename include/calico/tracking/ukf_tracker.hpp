#ifndef CALICO_TRACKING_UKF_TRACKER_HPP
#define CALICO_TRACKING_UKF_TRACKER_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include "calico/tracking/track.hpp"
#include "calico/utils/message_converter.hpp"
#include "calico/fusion/hungarian_matcher.hpp"

namespace calico {
namespace tracking {

/**
 * @brief UKF tracker configuration
 */
struct UKFConfig {
    // Process noise parameters
    double q_pos = 0.1;      // Position process noise
    double q_vel = 0.1;      // Velocity process noise
    
    // Measurement noise parameters  
    double r_pos = 0.5;      // Position measurement noise
    
    // UKF parameters
    double alpha = 0.001;    // Spread of sigma points
    double beta = 2.0;       // Prior knowledge of distribution
    double kappa = 0.0;      // Secondary scaling parameter
    
    // Track management
    int max_age_before_deletion = 10;
    int min_hits_before_confirmation = 3;
    double max_association_distance = 2.0;
};

/**
 * @brief Multi-object tracker using Unscented Kalman Filter
 * 
 * Tracks multiple cones over time using UKF for state estimation
 * and Hungarian algorithm for data association
 */
class UKFTracker {
public:
    UKFTracker(const UKFConfig& config = UKFConfig());
    ~UKFTracker() = default;
    
    /**
     * @brief Update tracker with new detections
     * @param detections New cone detections
     * @param timestamp Current timestamp
     * @param imu_data Optional IMU data for motion compensation
     */
    void update(const std::vector<utils::Cone>& detections,
                double timestamp,
                const utils::IMUData* imu_data = nullptr);
    
    /**
     * @brief Get current tracked cones
     * @return Vector of tracked cones with IDs and states
     */
    std::vector<utils::Cone> getTrackedCones() const;
    
    /**
     * @brief Get all active tracks
     * @return Map of track ID to track object
     */
    const std::unordered_map<int, std::shared_ptr<Track>>& 
    getTracks() const { return tracks_; }
    
    /**
     * @brief Set UKF configuration
     * @param config New configuration
     */
    void setConfig(const UKFConfig& config) { config_ = config; }
    
    /**
     * @brief Reset tracker, removing all tracks
     */
    void reset();
    
private:
    /**
     * @brief Predict step for all tracks
     * @param dt Time since last update
     * @param imu_data Optional IMU data for motion compensation
     */
    void predict(double dt, const utils::IMUData* imu_data);
    
    /**
     * @brief Associate detections with existing tracks
     * @param detections New detections
     * @return Match result from Hungarian algorithm
     */
    fusion::MatchResult associate(const std::vector<utils::Cone>& detections);
    
    /**
     * @brief Update matched tracks with measurements
     * @param matches Matched pairs from association
     * @param detections Detection measurements
     */
    void updateTracks(const std::vector<std::pair<int, int>>& matches,
                     const std::vector<utils::Cone>& detections);
    
    /**
     * @brief Create new tracks for unmatched detections
     * @param unmatched_detections Indices of unmatched detections
     * @param detections All detections
     */
    void createNewTracks(const std::vector<int>& unmatched_detections,
                        const std::vector<utils::Cone>& detections);
    
    /**
     * @brief Delete old tracks that haven't been updated
     */
    void deleteLostTracks();
    
    /**
     * @brief Generate unique track ID
     * @return New track ID
     */
    int generateTrackId();
    
private:
    UKFConfig config_;
    std::unordered_map<int, std::shared_ptr<Track>> tracks_;
    std::unique_ptr<fusion::HungarianMatcher> matcher_;
    double last_timestamp_;
    int next_track_id_;
};

} // namespace tracking
} // namespace calico

#endif // CALICO_TRACKING_UKF_TRACKER_HPP