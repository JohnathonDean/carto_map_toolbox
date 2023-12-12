#include "carto_map/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "glog/logging.h"

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const cartographer::sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;
  for (const cartographer::sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - cartographer::common::Pow2(resolution) /
                                         (2. * cartographer::common::Pow2(max_scan_range)));
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  num_scans = 2 * num_angular_perturbations + 1;

  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const cartographer::sensor::PointCloud& point_cloud,
                                   const double resolution,
                                   const double angular_search_max_scan_range)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;
  for (const cartographer::sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
    if(max_scan_range > angular_search_max_scan_range){
      // LOG(INFO) << " max_scan_range switch from " << max_scan_range << " to " << angular_search_max_scan_range;
      max_scan_range = angular_search_max_scan_range;
    }

  }
  // LOG(INFO) << "max_scan_range " << max_scan_range;
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - cartographer::common::Pow2(resolution) /
                                         (2. * cartographer::common::Pow2(max_scan_range)));

  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  num_scans = 2 * num_angular_perturbations + 1;


  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                                   const cartographer::mapping::CellLimits& cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    for (const Eigen::Array2i& xy_index : scans[i]) {
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

std::vector<cartographer::sensor::PointCloud> GenerateRotatedScans(
    const cartographer::sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<cartographer::sensor::PointCloud> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);

  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(cartographer::sensor::TransformPointCloud(
        point_cloud, cartographer::transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

std::vector<DiscreteScan2D> DiscretizeScans(
    const cartographer::mapping::MapLimits& map_limits,
    const std::vector<cartographer::sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());
  for (const cartographer::sensor::PointCloud& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const cartographer::sensor::RangefinderPoint& point : scan) {
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.position.head<2>();
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  return discrete_scans;
}



float ComputeCandidateScore(const cartographer::mapping::ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    candidate_score += probability;
  }
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}



RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const RealTimeCorrelativeScanMatcherParam& options)
    : options_(options) {}

std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher2D::Match(
    const cartographer::transform::Rigid2d& initial_pose_estimate,
    const cartographer::sensor::PointCloud& point_cloud,
    const cartographer::mapping::Grid2D& grid,
    cartographer::transform::Rigid2d* pose_estimate,
    double angle_scaling_factor) const
{
  CHECK(pose_estimate != nullptr);
  CHECK(angle_scaling_factor > 0) << angle_scaling_factor << " is not greater than 0!";
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const cartographer::sensor::PointCloud rotated_point_cloud = cartographer::sensor::TransformPointCloud(
      point_cloud,
      cartographer::transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
      LOG_FIRST_N(INFO , 2) << "angle_scaling_factor " << angle_scaling_factor;
      LOG_FIRST_N(INFO , 2) << "angular_search_window " << options_.angular_search_window * angle_scaling_factor;
  SearchParameters search_parameters(
      options_.linear_search_window, options_.angular_search_window * angle_scaling_factor,
      rotated_point_cloud, grid.limits().resolution(), options_.angular_search_step_max_range);

  const std::vector<cartographer::sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = cartographer::transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const cartographer::mapping::Grid2D& grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    candidate.score = ComputeCandidateScore(
        static_cast<const cartographer::mapping::ProbabilityGrid&>(grid),
        discrete_scans[candidate.scan_index], candidate.x_index_offset,
        candidate.y_index_offset);

    candidate.score *=
        std::exp(-cartographer::common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight));
  }
}

