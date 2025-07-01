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
    // This is a simplified implementation of the Hungarian algorithm
    // For production, consider using dlib::max_cost_assignment or munkres-cpp
    
    int n_rows = cost_matrix.rows();
    int n_cols = cost_matrix.cols();
    int dim = std::max(n_rows, n_cols);
    
    // Create square cost matrix by padding with large values
    Eigen::MatrixXd square_cost(dim, dim);
    square_cost.fill(std::numeric_limits<double>::max() / 2);
    square_cost.block(0, 0, n_rows, n_cols) = cost_matrix;
    
    // Implement Hungarian algorithm steps
    // Step 1: Subtract row minimum from each row
    Eigen::MatrixXd working_matrix = square_cost;
    for (int i = 0; i < dim; ++i) {
        double row_min = working_matrix.row(i).minCoeff();
        if (row_min < std::numeric_limits<double>::max() / 2) {
            working_matrix.row(i).array() -= row_min;
        }
    }
    
    // Step 2: Subtract column minimum from each column
    for (int j = 0; j < dim; ++j) {
        double col_min = working_matrix.col(j).minCoeff();
        if (col_min < std::numeric_limits<double>::max() / 2) {
            working_matrix.col(j).array() -= col_min;
        }
    }
    
    // Simplified assignment (greedy approach for now)
    // TODO: Implement full Hungarian algorithm or integrate external library
    std::vector<int> assignments(n_rows, -1);
    std::vector<bool> col_used(n_cols, false);
    
    // Find minimum in each row
    for (int i = 0; i < n_rows; ++i) {
        double min_val = std::numeric_limits<double>::max();
        int min_col = -1;
        
        for (int j = 0; j < n_cols; ++j) {
            if (!col_used[j] && working_matrix(i, j) < min_val) {
                min_val = working_matrix(i, j);
                min_col = j;
            }
        }
        
        if (min_col >= 0) {
            assignments[i] = min_col;
            col_used[min_col] = true;
        }
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