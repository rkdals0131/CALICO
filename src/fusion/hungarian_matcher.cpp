#include "calico/fusion/hungarian_matcher.hpp"
#include <limits>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace calico {
namespace fusion {

MatchResult HungarianMatcher::match(const Eigen::MatrixXd& cost_matrix, double max_distance) {
    MatchResult result;
    
    if (cost_matrix.rows() == 0 || cost_matrix.cols() == 0) {
        // Empty cost matrix - no matches possible
        for (int i = 0; i < cost_matrix.rows(); ++i) {
            result.unmatched_detections.push_back(i);
        }
        for (int j = 0; j < cost_matrix.cols(); ++j) {
            result.unmatched_tracks.push_back(j);
        }
        return result;
    }
    
    // Solve Hungarian algorithm
    std::vector<int> assignments = solveHungarian(cost_matrix);
    
    // Filter matches based on maximum distance threshold
    result = filterMatches(assignments, cost_matrix, max_distance);
    
    return result;
}

Eigen::MatrixXd HungarianMatcher::computeCostMatrix(
    const std::vector<std::pair<double, double>>& detections,
    const std::vector<std::pair<double, double>>& tracks) {
    
    size_t n_detections = detections.size();
    size_t n_tracks = tracks.size();
    
    Eigen::MatrixXd cost_matrix(n_detections, n_tracks);
    
    // Compute Euclidean distances
    for (size_t i = 0; i < n_detections; ++i) {
        for (size_t j = 0; j < n_tracks; ++j) {
            double dx = detections[i].first - tracks[j].first;
            double dy = detections[i].second - tracks[j].second;
            cost_matrix(i, j) = std::sqrt(dx * dx + dy * dy);
        }
    }
    
    return cost_matrix;
}

std::vector<int> HungarianMatcher::solveHungarian(const Eigen::MatrixXd& cost_matrix) {
    // Use Google OR-Tools for optimal assignment
    int n_rows = cost_matrix.rows();
    int n_cols = cost_matrix.cols();
    
    // Create the assignment solver
    operations_research::SimpleLinearSumAssignment assignment;
    
    // Scale costs to integers (OR-Tools works with integer costs)
    const int64_t scale_factor = 1000000; // Scale to preserve 6 decimal places
    
    // Add arcs with costs
    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            // Scale and convert to integer
            int64_t cost = static_cast<int64_t>(cost_matrix(i, j) * scale_factor);
            assignment.AddArcWithCost(i, j, cost);
        }
    }
    
    // Solve the assignment problem
    operations_research::SimpleLinearSumAssignment::Status status = assignment.Solve();
    
    std::vector<int> assignments(n_rows, -1);
    
    if (status == operations_research::SimpleLinearSumAssignment::OPTIMAL) {
        // Extract assignments
        for (int i = 0; i < n_rows; ++i) {
            assignments[i] = assignment.RightMate(i);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("hungarian_matcher"), 
                    "Hungarian algorithm did not find optimal solution");
    }
    
    return assignments;
}

MatchResult HungarianMatcher::filterMatches(const std::vector<int>& assignments,
                                           const Eigen::MatrixXd& cost_matrix,
                                           double max_distance) {
    MatchResult result;
    
    // Track which detections and tracks were matched
    std::vector<bool> det_matched(cost_matrix.rows(), false);
    std::vector<bool> track_matched(cost_matrix.cols(), false);
    
    // Process assignments
    for (size_t i = 0; i < assignments.size(); ++i) {
        int j = assignments[i];
        
        if (j >= 0 && cost_matrix(i, j) < max_distance) {
            // Valid match within threshold
            result.matches.emplace_back(i, j);
            det_matched[i] = true;
            track_matched[j] = true;
        }
    }
    
    // Find unmatched detections
    for (size_t i = 0; i < det_matched.size(); ++i) {
        if (!det_matched[i]) {
            result.unmatched_detections.push_back(i);
        }
    }
    
    // Find unmatched tracks
    for (size_t j = 0; j < track_matched.size(); ++j) {
        if (!track_matched[j]) {
            result.unmatched_tracks.push_back(j);
        }
    }
    
    return result;
}

} // namespace fusion
} // namespace calico