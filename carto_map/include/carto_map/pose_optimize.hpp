#include "cartographer/transform/transform.h"
#include <cartographer/mapping/pose_graph_interface.h>

#include <ceres/ceres.h>

typedef cartographer::mapping::PoseGraphInterface::Constraint::Pose PoseConstraint;

namespace carto_map {

// Ceres使用自动求导来得到梯度，因此在提供损失函数的时候，只要这样给一个模板就行了。
// 在这个函数中，start和end是两个坐标系的二维坐标和朝向角
// 而relative_pose表示他们的期望相对位姿。
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const cartographer::transform::Rigid2d& relative_pose, const T* const start,
    const T* const end) {
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0];
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose.translation().x()) - h[0],
           T(relative_pose.translation().y()) - h[1],
           cartographer::common::NormalizeAngleDifference(
               T(relative_pose.rotation().angle()) - h[2])}};
}

// 这个函数对上面的函数做了一个加权
template <typename T>
std::array<T, 3> ScaleError(const std::array<T, 3>& error,
                            double translation_weight, double rotation_weight)
{
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * rotation_weight
  }};
}

// 对上述函数做一个封装
class SpaCostFunction2D {
public:
    explicit SpaCostFunction2D(
            const PoseConstraint& observed_relative_pose)
        : rel_pose_cons_(observed_relative_pose) {}

    template <typename T>
    bool operator()(const T* const start_pose, const T* const end_pose,
                    T* e) const {
        const std::array<T, 3> error =
                ScaleError(ComputeUnscaledError(
                               cartographer::transform::Project2D(rel_pose_cons_.zbar_ij),
                               start_pose, end_pose),
                           rel_pose_cons_.translation_weight,
                           rel_pose_cons_.rotation_weight);
        std::copy(std::begin(error), std::end(error), e);
        return true;
    }

private:
    const PoseConstraint rel_pose_cons_;
};

// 可以看到，Ceres的CostFunction是对原始损失函数的自动求导（AutoDiff）封装。
ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const PoseConstraint& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}


void OptimizePose(
    std::vector<std::array<double, 3>>& pose,
    const std::vector<cartographer::transform::Rigid3d>& tf) {
  ceres::Problem optimize_problem;
  double trans_w = 100.;
  double rot_w = 1.;

  for (int i = 1; i < pose.size(); i++) {
    PoseConstraint rel_pose;  // 相对前一个坐标系的位姿
    rel_pose.zbar_ij = tf[i - 1].inverse() * tf[i];
    rel_pose.rotation_weight = rot_w;
    rel_pose.translation_weight = trans_w;

    // 把pose[i]的数据加入优化
    // pose[i]会被更新为优化后的结果。
    optimize_problem.AddResidualBlock(CreateAutoDiffSpaCostFunction(rel_pose),
                                      nullptr, pose[i - 1].data(),
                                      pose[i].data());
  }

  // 将第一个和最后一个坐标系的位置固定
  optimize_problem.SetParameterBlockConstant(pose.front().data());
  optimize_problem.SetParameterBlockConstant(pose.back().data());

  ceres::Solver::Summary sum;
  ceres::Solver::Options options;
  options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &optimize_problem, &sum);  // 求解优化问题
}

}  // namespace carto_map
