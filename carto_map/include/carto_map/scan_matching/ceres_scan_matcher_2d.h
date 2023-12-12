#ifndef SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

struct CeresScanMatcherParam {
  double occupied_space_weight;
  double translation_weight;
  double rotation_weight;

  bool use_nonmonotonic_steps;
  int max_num_iterations;
  int num_threads;
};

// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const CeresScanMatcherParam& options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector2d& target_translation,
             const cartographer::transform::Rigid2d& initial_pose_estimate,
             const cartographer::sensor::PointCloud& point_cloud,
             const cartographer::mapping::Grid2D& grid,
             cartographer::transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) /* const */;

 private:
  CeresScanMatcherParam options_;
  absl::Mutex mutex_;
};


#endif  // SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
