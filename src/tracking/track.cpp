#include "calico/tracking/track.hpp"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Cholesky>

namespace calico {
namespace tracking {

Track::Track(int track_id, double initial_x, double initial_y, double initial_z)
    : track_id_(track_id), age_(0), hit_count_(0), time_since_update_(0), z_(initial_z) {
    
    // Initialize UKF parameters
    initializeUKF(initial_x, initial_y);
    
    // Initialize color history
    color_history_.clear();
}

void Track::initializeUKF(double x, double y) {
    // State: [x, y, vx, vy]
    state_ = Eigen::Vector4d(x, y, 0.0, 0.0);
    
    // Initial covariance (matching Python defaults)
    covariance_ = Eigen::Matrix4d::Identity();
    covariance_.block<2, 2>(0, 0) *= 0.001;  // P_initial_pos = 0.001
    covariance_.block<2, 2>(2, 2) *= 100.0;  // P_initial_vel = 100.0
    
    // UKF parameters
    alpha_ = 0.001;
    beta_ = 2.0;
    kappa_ = 0.0;
    lambda_ = alpha_ * alpha_ * (STATE_DIM + kappa_) - STATE_DIM;
    
    // Compute weights
    computeWeights();
    
    // Process noise
    Q_ = Eigen::Matrix4d::Identity();
    Q_.block<2, 2>(0, 0) *= 0.1;  // Position process noise
    Q_.block<2, 2>(2, 2) *= 1.0;  // Velocity process noise
    
    // Measurement noise
    R_ = Eigen::Matrix2d::Identity() * 0.5;
}

void Track::computeWeights() {
    double weight_0 = lambda_ / (STATE_DIM + lambda_);
    weights_mean_ = Eigen::VectorXd(2 * STATE_DIM + 1);
    weights_cov_ = Eigen::VectorXd(2 * STATE_DIM + 1);
    
    weights_mean_(0) = weight_0;
    weights_cov_(0) = weight_0 + (1 - alpha_ * alpha_ + beta_);
    
    for (int i = 1; i < 2 * STATE_DIM + 1; ++i) {
        weights_mean_(i) = 0.5 / (STATE_DIM + lambda_);
        weights_cov_(i) = weights_mean_(i);
    }
}

void Track::predict(double dt, double ax, double ay) {
    // Generate sigma points
    Eigen::MatrixXd sigma_points = generateSigmaPoints();
    
    // Predict sigma points
    Eigen::MatrixXd predicted_sigma = predictSigmaPoints(sigma_points, dt, ax, ay);
    
    // Compute predicted mean
    state_.setZero();
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        state_ += weights_mean_(i) * predicted_sigma.col(i);
    }
    
    // Compute predicted covariance
    covariance_.setZero();
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        Eigen::Vector4d diff = predicted_sigma.col(i) - state_;
        covariance_ += weights_cov_(i) * (diff * diff.transpose());
    }
    covariance_ += Q_;
    
    // Increment time since update
    time_since_update_++;
    age_++;
}

Eigen::MatrixXd Track::generateSigmaPoints() {
    Eigen::MatrixXd sigma_points(STATE_DIM, 2 * STATE_DIM + 1);
    
    // Compute square root of covariance
    Eigen::Matrix4d sqrt_cov = ((STATE_DIM + lambda_) * covariance_).llt().matrixL();
    
    // First sigma point is the mean
    sigma_points.col(0) = state_;
    
    // Generate other sigma points
    for (int i = 0; i < STATE_DIM; ++i) {
        sigma_points.col(i + 1) = state_ + sqrt_cov.col(i);
        sigma_points.col(i + 1 + STATE_DIM) = state_ - sqrt_cov.col(i);
    }
    
    return sigma_points;
}

Eigen::MatrixXd Track::predictSigmaPoints(const Eigen::MatrixXd& sigma_points,
                                         double dt, double ax, double ay) {
    Eigen::MatrixXd predicted(STATE_DIM, 2 * STATE_DIM + 1);
    
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        double x = sigma_points(0, i);
        double y = sigma_points(1, i);
        double vx = sigma_points(2, i);
        double vy = sigma_points(3, i);
        
        // Motion model
        predicted(0, i) = x + vx * dt + 0.5 * ax * dt * dt;
        predicted(1, i) = y + vy * dt + 0.5 * ay * dt * dt;
        predicted(2, i) = vx + ax * dt;
        predicted(3, i) = vy + ay * dt;
    }
    
    return predicted;
}

void Track::update(double x, double y, double z, const std::string& color) {
    // Update z directly (no filtering)
    z_ = z;
    
    // Measurement
    Eigen::Vector2d measurement(x, y);
    
    // Generate sigma points
    Eigen::MatrixXd sigma_points = generateSigmaPoints();
    
    // Transform sigma points to measurement space
    Eigen::MatrixXd sigma_z(MEAS_DIM, 2 * STATE_DIM + 1);
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        sigma_z(0, i) = sigma_points(0, i);  // x
        sigma_z(1, i) = sigma_points(1, i);  // y
    }
    
    // Compute predicted measurement mean
    Eigen::Vector2d z_pred = Eigen::Vector2d::Zero();
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        z_pred += weights_mean_(i) * sigma_z.col(i);
    }
    
    // Compute innovation covariance
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        Eigen::Vector2d diff = sigma_z.col(i) - z_pred;
        S += weights_cov_(i) * (diff * diff.transpose());
    }
    S += R_;
    
    // Compute cross covariance
    Eigen::Matrix<double, 4, 2> Pxz = Eigen::Matrix<double, 4, 2>::Zero();
    for (int i = 0; i < 2 * STATE_DIM + 1; ++i) {
        Eigen::Vector4d x_diff = sigma_points.col(i) - state_;
        Eigen::Vector2d z_diff = sigma_z.col(i) - z_pred;
        Pxz += weights_cov_(i) * (x_diff * z_diff.transpose());
    }
    
    // Kalman gain (using LU decomposition to avoid inverse)
    Eigen::Matrix<double, 4, 2> K = Pxz * S.lu().solve(Eigen::Matrix2d::Identity());
    
    // Update state and covariance
    Eigen::Vector2d innovation = measurement - z_pred;
    state_ += K * innovation;
    covariance_ -= K * S * K.transpose();
    
    // Update tracking info
    time_since_update_ = 0;
    hit_count_++;
    
    // Update color history
    updateColorHistory(color);
}

void Track::updateColorHistory(const std::string& color) {
    if (color != "Unknown") {
        color_history_.push_back(color);
        if (color_history_.size() > MAX_COLOR_HISTORY) {
            color_history_.pop_front();
        }
    }
}

Eigen::Vector3d Track::getPosition() const {
    return Eigen::Vector3d(state_(0), state_(1), z_);
}

Eigen::Vector2d Track::getVelocity() const {
    return Eigen::Vector2d(state_(2), state_(3));
}

std::string Track::getColor() const {
    if (color_history_.empty()) {
        return "Unknown";
    }
    
    // Count occurrences of each color
    std::unordered_map<std::string, int> color_count;
    for (const auto& color : color_history_) {
        color_count[color]++;
    }
    
    // Find most common color (excluding "unknown")
    std::string best_color = "unknown";
    int max_count = 0;
    for (const auto& [color, count] : color_count) {
        if (color != "unknown" && count > max_count) {
            max_count = count;
            best_color = color;
        }
    }
    
    // Capitalize first letter to match Python output
    if (best_color != "unknown" && !best_color.empty()) {
        std::string result = best_color;
        result[0] = std::toupper(result[0]);
        // Handle "blue cone" -> "Blue cone" format
        for (size_t i = 1; i < result.length(); ++i) {
            if (result[i-1] == ' ' && i < result.length()) {
                result[i] = std::toupper(result[i]);
            }
        }
        return result;
    }
    
    return "Unknown";
}

} // namespace tracking
} // namespace calico