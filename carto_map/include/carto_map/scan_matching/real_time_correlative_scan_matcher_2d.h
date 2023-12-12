#ifndef SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
#define SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "Eigen/Core"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "cartographer/common/math.h"

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

// Describes the search space.
struct SearchParameters {
    // Linear search window in pixel offsets; bounds are inclusive.
    struct LinearBounds {
        int min_x;
        int max_x;
        int min_y;
        int max_y;
    };

    SearchParameters(double linear_search_window, double angular_search_window,
                    const cartographer::sensor::PointCloud& point_cloud, double resolution);

    SearchParameters(double linear_search_window, double angular_search_window,
                    const cartographer::sensor::PointCloud& point_cloud, double resolution,
                    double angular_search_max_scan_range);
    // For testing.
    SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                    double angular_perturbation_step_size, double resolution);

    // Tightens the search window as much as possible.
    void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                    const cartographer::mapping::CellLimits& cell_limits);

    int num_angular_perturbations;
    double angular_perturbation_step_size;
    double resolution;
    int num_scans;
    std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
std::vector<cartographer::sensor::PointCloud> GenerateRotatedScans(
    const cartographer::sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan2D> DiscretizeScans(
    const cartographer::mapping::MapLimits& map_limits,
    const std::vector<cartographer::sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index,
              const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

struct RealTimeCorrelativeScanMatcherParam {
    double linear_search_window;
    double angular_search_window;
    double translation_delta_cost_weight;
    double rotation_delta_cost_weight;
    double angular_search_step_max_range;
};

class RealTimeCorrelativeScanMatcher2D {
 public:
    explicit RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcherParam& options);

    RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcher2D&) = delete;
    RealTimeCorrelativeScanMatcher2D& operator=(const RealTimeCorrelativeScanMatcher2D&) = delete;

    double Match(const cartographer::transform::Rigid2d& initial_pose_estimate,
                const cartographer::sensor::PointCloud& point_cloud,
                const cartographer::mapping::Grid2D& grid,
                cartographer::transform::Rigid2d* pose_estimate,double Angle_scaling_factor) const;

    void ScoreCandidates(const cartographer::mapping::Grid2D& grid,
                        const std::vector<DiscreteScan2D>& discrete_scans,
                        const SearchParameters& search_parameters,
                        std::vector<Candidate2D>* candidates) const;

 private:
    std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(
        const SearchParameters& search_parameters) const;

    RealTimeCorrelativeScanMatcherParam options_;
};

#endif  // SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
