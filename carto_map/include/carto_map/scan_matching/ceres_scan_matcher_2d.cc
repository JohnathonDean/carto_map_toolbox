#include "carto_map/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "glog/logging.h"


class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const cartographer::sensor::PointCloud& point_cloud,
                              const cartographer::mapping::Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const cartographer::mapping::MapLimits& limits = grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  static constexpr float kMinProbability = 0.1f;
  static constexpr float kMaxProbability = 1.f - kMinProbability;
  static constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
  static constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const cartographer::mapping::Grid2D& grid) : grid_(grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } else {
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const cartographer::mapping::Grid2D& grid_;
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const cartographer::sensor::PointCloud& point_cloud_;
  const cartographer::mapping::Grid2D& grid_;
};

ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor,
    const cartographer::sensor::PointCloud& point_cloud,
    const cartographer::mapping::Grid2D& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size());
}

class TranslationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
};

class RotationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

CeresScanMatcher2D::CeresScanMatcher2D(
    const CeresScanMatcherParam& options)
    : options_(options) {
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                            const cartographer::transform::Rigid2d& initial_pose_estimate,
                            const cartographer::sensor::PointCloud& point_cloud,
                            const cartographer::mapping::Grid2D& grid,
                            cartographer::transform::Rigid2d* pose_estimate,
                            ceres::Solver::Summary* summary) /* const */ {
    absl::MutexLock locker(&mutex_);
    double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                    initial_pose_estimate.translation().y(),
                                    initial_pose_estimate.rotation().angle()};
    ceres::Problem problem;
    CHECK_GT(options_.occupied_space_weight, 0.);

    problem.AddResidualBlock(
        CreateOccupiedSpaceCostFunction2D(options_.occupied_space_weight / std::sqrt(static_cast<double>(point_cloud.size())),
                                        point_cloud, grid),
        nullptr /* loss function */,
        ceres_pose_estimate);

    CHECK_GT(options_.translation_weight, 0.);
    problem.AddResidualBlock(
        TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(options_.translation_weight, target_translation),
        nullptr /* loss function */,
        ceres_pose_estimate);

    CHECK_GT(options_.rotation_weight, 0.);
    problem.AddResidualBlock(
        RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(options_.rotation_weight, ceres_pose_estimate[2]),
        nullptr /* loss function */,
        ceres_pose_estimate);

    ceres::Solver::Options ceres_solver_options_;
    ceres_solver_options_.use_nonmonotonic_steps = options_.use_nonmonotonic_steps;
    ceres_solver_options_.max_num_iterations = options_.max_num_iterations;
    ceres_solver_options_.num_threads = options_.num_threads;
    ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;

    ceres::Solve(ceres_solver_options_, &problem, summary);

    *pose_estimate = cartographer::transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

