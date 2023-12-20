#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "carto_map/outlier_filter.hpp"
#include "carto_map/voxel_filter.hpp"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

class PointcloudMapMatch {
 public:
  PointcloudMapMatch();
  ~PointcloudMapMatch();

  pcl::PointCloud<pcl::PointXYZ> GetPointcloudFromGrid(
      const cartographer::mapping::Grid2D& grid);

  Eigen::Matrix4f FromRigid2d(const cartographer::transform::Rigid2d& pose);

  cartographer::transform::Rigid3d ToRigid3d(
      const Eigen::Matrix4f& pose_matrix);

  void MatchICP(const cartographer::mapping::Grid2D& input_grid,
                const cartographer::mapping::Grid2D& source_grid,
                const cartographer::transform::Rigid2d& predict_pose,
                cartographer::transform::Rigid2d* pose_estimate);

  void MatchNDT(const cartographer::mapping::Grid2D& input_grid,
                const cartographer::mapping::Grid2D& source_grid,
                const cartographer::transform::Rigid2d& predict_pose,
                cartographer::transform::Rigid2d* pose_estimate);

  void SaveImageFromMatchPointcloud(
      const pcl::PointCloud<pcl::PointXYZ>& input,
      const pcl::PointCloud<pcl::PointXYZ>& source,
      const Eigen::Matrix4f& trans_matrix,
      const std::string& file_name) {
    cv::Mat image(6000, 6000, CV_8UC3, cv::Scalar(255, 255, 255));

    pcl::PointCloud<pcl::PointXYZ> input_trans = input;
    pcl::transformPointCloud(input, input_trans, trans_matrix);

    for (const auto& point : input_trans.points) {
      int x = static_cast<int>(point.x * 100 + image.cols / 2);
      int y = static_cast<int>(point.y * 100 + image.rows / 2);
      cv::circle(image, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
    }
    for (const auto& point : source.points) {
      int x = static_cast<int>(point.x * 100 + image.cols / 2);
      int y = static_cast<int>(point.y * 100 + image.rows / 2);
      cv::circle(image, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
    }

    cv::imwrite(file_name, image);
  }

  void SaveImageFromMatchGrid(
      const cartographer::mapping::Grid2D& input_grid,
      const cartographer::mapping::Grid2D& source_grid,
      const cartographer::transform::Rigid3d& input_pose,
      const cartographer::transform::Rigid3d& source_pose,
      const std::string& file_name) {
    pcl::PointCloud<pcl::PointXYZ> input = GetPointcloudFromGrid(input_grid);
    pcl::PointCloud<pcl::PointXYZ> source = GetPointcloudFromGrid(source_grid);
    Eigen::Matrix4f trans_matrix = FromRigid2d(
        cartographer::transform::Project2D(source_pose.inverse() * input_pose));

    SaveImageFromMatchPointcloud(input, source, trans_matrix, file_name);
  }

 private:
  /* data */
};

PointcloudMapMatch::PointcloudMapMatch() {}

PointcloudMapMatch::~PointcloudMapMatch() {}

pcl::PointCloud<pcl::PointXYZ> PointcloudMapMatch::GetPointcloudFromGrid(
    const cartographer::mapping::Grid2D& grid) {
  pcl::PointCloud<pcl::PointXYZ> pointcloud_res;
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
      pointcloud_res.push_back(pcl::PointXYZ(point_x, point_y, 0));
    }
  }
  return pointcloud_res;
}

Eigen::Matrix4f PointcloudMapMatch::FromRigid2d(
    const cartographer::transform::Rigid2d& pose) {
  Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
  pose_matrix.block<2, 2>(0, 0) =
      pose.rotation().toRotationMatrix().cast<float>();
  float pose_x = pose.translation().x();
  float pose_y = pose.translation().y();
  pose_matrix.block<2, 1>(0, 3) << pose_x, pose_y;
  return pose_matrix;
}

cartographer::transform::Rigid3d PointcloudMapMatch::ToRigid3d(
    const Eigen::Matrix4f& pose_matrix) {
  Eigen::Matrix3f rotation_matrix = pose_matrix.block<3, 3>(0, 0);
  Eigen::Vector3f translation_vector = pose_matrix.block<3, 1>(0, 3);
  cartographer::transform::Rigid3d pose(
      cartographer::transform::Rigid3d::Vector(translation_vector.x(),
                                               translation_vector.y(),
                                               translation_vector.z()),
      Eigen::Quaterniond(rotation_matrix.cast<double>()));
  return pose;
}

void PointcloudMapMatch::MatchICP(
    const cartographer::mapping::Grid2D& input_grid,
    const cartographer::mapping::Grid2D& source_grid,
    const cartographer::transform::Rigid2d& predict_pose,
    cartographer::transform::Rigid2d* pose_estimate) {
  pcl::PointCloud<pcl::PointXYZ> cloud_input = GetPointcloudFromGrid(source_grid);

  cartographer::transform::Rigid2d trans_pose({0.2,0.3}, 0.3);
  Eigen::Matrix4f trans_matrix = FromRigid2d(trans_pose);
  pcl::PointCloud<pcl::PointXYZ> cloud_trans = cloud_input;
  pcl::transformPointCloud(cloud_input, cloud_trans, trans_matrix);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(100);
  icp.setInputSource(cloud_trans.makeShared());
  icp.setInputTarget(cloud_input.makeShared());

  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  icp.align(final_cloud);
  LOG(INFO) << "MatchICP has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore();
  if (icp.hasConverged()) {
    Eigen::Matrix4f final_matrix = icp.getFinalTransformation();
    std::cout << "predict transformation " << std::endl << trans_matrix << std::endl;
    std::cout << "final transformation " << std::endl << final_matrix << std::endl;
    // SaveImageFromMatchPointcloud(cloud_input, cloud_trans, trans_matrix, "/home/dean/Pictures/submap1.png");
    // SaveImageFromMatchPointcloud(cloud_input, cloud_trans, final_matrix, "/home/dean/Pictures/submap2.png");
  }



  // pcl::PointCloud<pcl::PointXYZ> cloud_source =
  //     GetPointcloudFromGrid(input_grid);
  // Eigen::Matrix4f predict_matrix = FromRigid2d(predict_pose);

  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setMaximumIterations(100);
  // icp.setInputSource(cloud_source.makeShared());
  // icp.setInputTarget(cloud_input.makeShared());

  // pcl::PointCloud<pcl::PointXYZ> final_cloud;
  // icp.align(final_cloud, predict_matrix);
  // LOG(INFO) << "MatchICP has converged:" << icp.hasConverged()
  //           << " score: " << icp.getFitnessScore();

  // if (icp.hasConverged()) {
  //   Eigen::Matrix4f trans_matrix = icp.getFinalTransformation();
  //   std::cout << "predict transformation " << std::endl << predict_matrix << std::endl;
  //   std::cout << "final transformation " << std::endl << trans_matrix << std::endl;
  //   SaveImageFromMatchPointcloud(cloud_input, cloud_source, predict_matrix, "/home/dean/Pictures/submap1.png");
  //   SaveImageFromMatchPointcloud(cloud_input, cloud_source, trans_matrix, "/home/dean/Pictures/submap2.png");
  //   *pose_estimate = cartographer::transform::Project2D(ToRigid3d(trans_matrix));
  // } else {
  //   *pose_estimate = predict_pose;
  // }
}
