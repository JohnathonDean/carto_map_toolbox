#include <opencv2/opencv.hpp>

#include "carto_map/scan_matching/ceres_scan_matcher_2d.h"
#include "carto_map/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "carto_map/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "carto_map/voxel_filter.hpp"
#include "carto_map/outlier_filter.hpp"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

class SubmapMatcher {
 public:
  SubmapMatcher() {
    correlative_scan_matcher_options.linear_search_window = 0.1;
    correlative_scan_matcher_options.angular_search_window = 1.57;
    correlative_scan_matcher_options.translation_delta_cost_weight = 1e-1;
    correlative_scan_matcher_options.rotation_delta_cost_weight = 1e-1;
    correlative_scan_matcher_options.angular_search_step_max_range = 20;

    ceres_scan_matcher_options.occupied_space_weight = 1.0;
    ceres_scan_matcher_options.translation_weight = 10.0;
    ceres_scan_matcher_options.rotation_weight = 40.0;
    ceres_scan_matcher_options.use_nonmonotonic_steps = false;
    ceres_scan_matcher_options.max_num_iterations = 20;
    ceres_scan_matcher_options.num_threads = 3;

    fast_correlative_scan_matcher_options.linear_search_window = 0.5;
    fast_correlative_scan_matcher_options.angular_search_window = 0.6;
    fast_correlative_scan_matcher_options.branch_and_bound_depth = 7;

    real_time_correlative_scan_matcher_ =
        std::make_shared<RealTimeCorrelativeScanMatcher2D>(
            correlative_scan_matcher_options);
    ceres_scan_matcher_ =
        std::make_shared<CeresScanMatcher2D>(ceres_scan_matcher_options);
  }
  ~SubmapMatcher() {}

  void MatchRT(const cartographer::mapping::Grid2D& input_grid,
               const cartographer::mapping::Grid2D& source_grid,
               const cartographer::transform::Rigid2d& predict_pose,
               cartographer::transform::Rigid2d* pose_estimate);

  void MatchCSM(const cartographer::mapping::Grid2D& input_grid,
                const cartographer::mapping::Grid2D& source_grid,
                const cartographer::transform::Rigid2d& predict_pose,
                cartographer::transform::Rigid2d* pose_estimate);

  void MatchFullCSM(const cartographer::mapping::Grid2D& input_grid,
                    const cartographer::mapping::Grid2D& source_grid,
                    const cartographer::transform::Rigid2d& predict_pose,
                    cartographer::transform::Rigid2d* pose_estimate);

  cartographer::sensor::PointCloud GetPointcloudFromGrid(
      const cartographer::mapping::Grid2D& grid);

  cartographer::sensor::PointCloud GetCroppedPointcloud(
      const cartographer::sensor::PointCloud& input_points,
      const cartographer::mapping::Grid2D& source_grid,
      const cartographer::transform::Rigid2d& predict_pose);

  void SaveImageFromGrid(const cartographer::mapping::Grid2D& grid,
                         const std::string& file_name) {
    cv::Mat image(1000, 1000, CV_8U, cv::Scalar(255));

    Eigen::Array2i offset;
    cartographer::mapping::CellLimits cell_limits;
    grid.ComputeCroppedLimits(&offset, &cell_limits);
    if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
      LOG(WARNING) << "Empty grid!";
      return;
    }

    for (const Eigen::Array2i& xy_index :
         cartographer::mapping::XYIndexRangeIterator(cell_limits)) {
      const Eigen::Array2i index = xy_index + offset;
      if (!grid.IsKnown(index)) continue;
      // std::cout << grid.GetCorrespondenceCost(index) << "/" <<
      // grid.GetMaxCorrespondenceCost() << ";";
      if (1.f - grid.GetCorrespondenceCost(index) > 0.5) {
        int x = static_cast<int>(500 + index.x());
        int y = static_cast<int>(500 + index.y());
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
          image.at<uchar>(y, x) = 0;  // 设置像素值为黑色
        }
      }
    }
    cv::imwrite(file_name, image);
  }

  void SaveImageFromMatchGrid(
      const cartographer::mapping::Grid2D& input_grid,
      const cartographer::mapping::Grid2D& source_grid,
      const cartographer::transform::Rigid3d& input_pose,
      const cartographer::transform::Rigid3d& source_pose,
      const std::string& file_name) {
    cv::Mat image(6000, 6000, CV_8UC3, cv::Scalar(255, 255, 255));

    cartographer::sensor::PointCloud input = GetPointcloudFromGrid(input_grid);

    input = VoxelFilter(input, 0.05);
    input = StatisticalOutlierFilterPCL(input, 50, 1.0);

    cartographer::sensor::PointCloud input1 = GetCroppedPointcloud(
        input, source_grid,
        cartographer::transform::Project2D(source_pose.inverse() * input_pose));
    cartographer::sensor::PointCloud input_trans =
        cartographer::sensor::TransformPointCloud(
            input1, (source_pose.inverse() * input_pose).cast<float>());
    cartographer::sensor::PointCloud source =
        GetPointcloudFromGrid(source_grid);
    // cartographer::sensor::PointCloud source_trans =
    // cartographer::sensor::TransformPointCloud(source,
    // source_pose.cast<float>()); 遍历点云数据并绘制到图像上
    for (const auto& point : input_trans.points()) {
      int x = static_cast<int>(
          point.position.x() * 100 +
          image.cols / 2);  // 假设放大了 100 倍，并将原点平移到图像中心
      int y = static_cast<int>(point.position.y() * 100 + image.rows / 2);
      cv::circle(image, cv::Point(x, y), 1, cv::Scalar(0, 0, 255),
                 -1);  // -1 表示实心圆 红色
    }
    // for (const auto& point : source.points()) {
    //   int x = static_cast<int>(
    //       point.position.x() * 100 +
    //       image.cols / 2);  // 假设放大了 100 倍，并将原点平移到图像中心
    //   int y = static_cast<int>(point.position.y() * 100 + image.rows / 2);
    //   cv::circle(image, cv::Point(x, y), 1, cv::Scalar(255, 0, 0),
    //              -1);  // -1 表示实心圆 蓝色
    // }

    cv::imwrite(file_name, image);
  }

 private:
  RealTimeCorrelativeScanMatcherParam correlative_scan_matcher_options;
  CeresScanMatcherParam ceres_scan_matcher_options;
  FastCorrelativeScanMatcherParam fast_correlative_scan_matcher_options;

  std::shared_ptr<RealTimeCorrelativeScanMatcher2D>
      real_time_correlative_scan_matcher_;
  std::shared_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
  std::shared_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher_;
};

cartographer::sensor::PointCloud SubmapMatcher::GetPointcloudFromGrid(
    const cartographer::mapping::Grid2D& grid) {
  cartographer::sensor::PointCloud pointcloud_res;
  Eigen::Array2i offset;
  cartographer::mapping::CellLimits cell_limits;
  grid.ComputeCroppedLimits(&offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    LOG(WARNING) << "Empty grid!";
    return pointcloud_res;
  }

  for (const Eigen::Array2i& xy_index :
       cartographer::mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i index = xy_index + offset;
    if (!grid.IsKnown(index)) continue;
    if (1.f - grid.GetCorrespondenceCost(index) > 0.5) {
      double point_x = grid.limits().max().x() -
                       grid.limits().resolution() * (index.y() + 0.5);
      double point_y = grid.limits().max().y() -
                       grid.limits().resolution() * (index.x() + 0.5);
      Eigen::Vector3f point_pose(point_x, point_y, 0);
      pointcloud_res.push_back({point_pose});
    }
  }
  return pointcloud_res;
  // return VoxelFilter(pointcloud_res, 0.05);
}

cartographer::sensor::PointCloud SubmapMatcher::GetCroppedPointcloud(
    const cartographer::sensor::PointCloud& input_points,
    const cartographer::mapping::Grid2D& source_grid,
    const cartographer::transform::Rigid2d& predict_pose) {
  Eigen::Array2i tar_offset;
  cartographer::mapping::CellLimits tar_cell_limits;
  source_grid.ComputeCroppedLimits(&tar_offset, &tar_cell_limits);
  if (tar_cell_limits.num_x_cells == 0 || tar_cell_limits.num_y_cells == 0)
    return input_points;
  // submap裁剪后的boundbox的边界参数
  double rect_y_max =
      source_grid.limits().max().y() -
      source_grid.limits().resolution() * (tar_offset.x() + 0 + 0.5);
  double rect_y_min = source_grid.limits().max().y() -
                      source_grid.limits().resolution() *
                          (tar_offset.x() + tar_cell_limits.num_x_cells + 0.5);
  double rect_x_max =
      source_grid.limits().max().x() -
      source_grid.limits().resolution() * (tar_offset.y() + 0 + 0.5);
  double rect_x_min = source_grid.limits().max().x() -
                      source_grid.limits().resolution() *
                          (tar_offset.y() + tar_cell_limits.num_y_cells + 0.5);
  // LOG(INFO) << "Source Grid2D bound_box x:[" << rect_x_min << "," << rect_x_max
  //           << "] y:[" << rect_y_min << "," << rect_y_max << "]";
  cartographer::sensor::PointCloud res_points;
  for (size_t i = 0; i < input_points.size(); i++) {
    cartographer::sensor::RangefinderPoint point_tran =
        cartographer::transform::Embed3D(predict_pose).cast<float>() *
        input_points[i];
    // LOG(INFO) << "Point:[" << i << "]:(" << point_tran.position.x() << ","<< point_tran.position.y() << ")";
    if (point_tran.position.x() < rect_x_max && point_tran.position.x() > rect_x_min &&
        point_tran.position.y() < rect_y_max && point_tran.position.y() > rect_y_min) {
      res_points.push_back(input_points[i]);
    }
  }
  return res_points;
}

void SubmapMatcher::MatchCSM(
    const cartographer::mapping::Grid2D& input_grid,
    const cartographer::mapping::Grid2D& source_grid,
    const cartographer::transform::Rigid2d& predict_pose,
    cartographer::transform::Rigid2d* pose_estimate) {
  cartographer::sensor::PointCloud input_point_cloud = GetPointcloudFromGrid(input_grid);
  input_point_cloud = StatisticalOutlierFilterPCL(input_point_cloud, 60, 1.0);
  input_point_cloud = VoxelFilter(input_point_cloud, 0.075);
  input_point_cloud = GetCroppedPointcloud(input_point_cloud, source_grid, predict_pose);

  fast_correlative_scan_matcher_ =
      std::make_shared<FastCorrelativeScanMatcher2D>(
          source_grid, fast_correlative_scan_matcher_options);
  cartographer::transform::Rigid2d initial_ceres_pose = predict_pose;
  float score = 0.;
  fast_correlative_scan_matcher_->Match(predict_pose, input_point_cloud, 0.2,
                                        &score, &initial_ceres_pose);
  // fast_correlative_scan_matcher_->MatchFullSubmap(input_point_cloud, 0.2,
  // &score, &initial_ceres_pose);
  LOG(INFO) << "predict_pose" << predict_pose;
  LOG(INFO) << "fast_correlative_scan_matcher_" << initial_ceres_pose
            << "   score: " << score;

  cartographer::transform::Rigid2d pose_observation = predict_pose;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(predict_pose.translation(), initial_ceres_pose,
                             input_point_cloud, source_grid, &pose_observation,
                             &summary);
  LOG(INFO) << "ceres_scan_matcher" << pose_observation;

  *pose_estimate = pose_observation;
}

void SubmapMatcher::MatchFullCSM(
    const cartographer::mapping::Grid2D& input_grid,
    const cartographer::mapping::Grid2D& source_grid,
    const cartographer::transform::Rigid2d& predict_pose,
    cartographer::transform::Rigid2d* pose_estimate) {
  cartographer::sensor::PointCloud input_point_cloud =
      GetPointcloudFromGrid(input_grid);

  fast_correlative_scan_matcher_ =
      std::make_shared<FastCorrelativeScanMatcher2D>(
          source_grid, fast_correlative_scan_matcher_options);
  cartographer::transform::Rigid2d initial_ceres_pose = predict_pose;
  float score = 0.;
  // fast_correlative_scan_matcher_->Match(predict_pose, input_point_cloud, 0.2,
  // &score, &initial_ceres_pose);
  fast_correlative_scan_matcher_->MatchFullSubmap(input_point_cloud, 0.2,
                                                  &score, &initial_ceres_pose);
  LOG(INFO) << "predict_pose" << predict_pose;
  LOG(INFO) << "fast_correlative_scan_matcher_" << initial_ceres_pose
            << "   score: " << score;

  cartographer::transform::Rigid2d pose_observation = predict_pose;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(predict_pose.translation(), initial_ceres_pose,
                             input_point_cloud, source_grid, &pose_observation,
                             &summary);
  LOG(INFO) << "ceres_scan_matcher" << pose_observation;

  *pose_estimate = pose_observation;
}

void SubmapMatcher::MatchRT(
    const cartographer::mapping::Grid2D& input_grid,
    const cartographer::mapping::Grid2D& source_grid,
    const cartographer::transform::Rigid2d& predict_pose,
    cartographer::transform::Rigid2d* pose_estimate) {
  cartographer::sensor::PointCloud input_point_cloud =
      GetPointcloudFromGrid(input_grid);

  double angle_scaling_factor_ = 1.0;
  cartographer::transform::Rigid2d initial_ceres_pose = predict_pose;
  const double score = real_time_correlative_scan_matcher_->Match(
      predict_pose, input_point_cloud, source_grid, &initial_ceres_pose,
      angle_scaling_factor_);
  LOG(INFO) << "predict_pose" << predict_pose;
  LOG(INFO) << "real_time_correlative_scan_matcher" << initial_ceres_pose
            << "   score: " << score;

  cartographer::transform::Rigid2d pose_observation = predict_pose;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(predict_pose.translation(), initial_ceres_pose,
                             input_point_cloud, source_grid, &pose_observation,
                             &summary);
  LOG(INFO) << "ceres_scan_matcher" << pose_observation;

  *pose_estimate = pose_observation;
}
