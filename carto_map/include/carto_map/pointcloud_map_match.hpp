#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    cv::Mat image(10000, 10000, CV_8UC3, cv::Scalar(255, 255, 255));

    pcl::PointCloud<pcl::PointXYZ> input_trans = input;
    pcl::transformPointCloud(input, input_trans, trans_matrix);

    for (const auto& point : input_trans.points) {
      int x = static_cast<int>(point.x * 1000.0 + image.cols / 2.0);
      int y = static_cast<int>(point.y * 1000.0 + image.rows / 2.0);
      cv::circle(image, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1);
    }
    for (const auto& point : source.points) {
      int x = static_cast<int>(point.x * 1000.0 + image.cols / 2.0);
      int y = static_cast<int>(point.y * 1000.0 + image.rows / 2.0);
      cv::circle(image, cv::Point(x, y), 10, cv::Scalar(255, 0, 0), -1);
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
  Eigen::Matrix4f predict_matrix = FromRigid2d(predict_pose);
  pcl::PointCloud<pcl::PointXYZ> cloud_input = GetPointcloudFromGrid(input_grid);
  pcl::PointCloud<pcl::PointXYZ> cloud_source = GetPointcloudFromGrid(source_grid);

  pcl::PointCloud<pcl::PointXYZ> cloud_input_filtered = cloud_input;
  pcl::PointCloud<pcl::PointXYZ> cloud_source_filtered = cloud_source;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setMeanK(100);            //设置领域点的个数
  sor.setStddevMulThresh(1.0); //设置离群点的阈值
  sor.setInputCloud(cloud_input.makeShared());    //输入点云
  sor.filter(cloud_input_filtered); //滤波后的结果
  sor.setInputCloud(cloud_source.makeShared());    //输入点云
  sor.filter(cloud_source_filtered); //滤波后的结果
  cloud_input = cloud_input_filtered;
  cloud_source = cloud_source_filtered;

  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(300);
  icp.setMaxCorrespondenceDistance (0.005);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(1e-9);
  icp.setInputSource(cloud_input.makeShared());
  icp.setInputTarget(cloud_source.makeShared());

  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  icp.align(final_cloud, predict_matrix);
  LOG(INFO) << "MatchICP has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore();

  if (icp.hasConverged()) {
    Eigen::Matrix4f trans_matrix = icp.getFinalTransformation();
    std::cout << "predict transformation " << std::endl << predict_matrix << std::endl;
    std::cout << "final transformation " << std::endl << trans_matrix << std::endl;

    // pcl::PointCloud<pcl::PointXYZ> cloud_trans2 = cloud_input;
    // pcl::transformPointCloud(cloud_input, cloud_trans2, predict_matrix);
    // pcl::PointCloud<pcl::PointXYZ> cloud_trans3 = cloud_input;
    // pcl::transformPointCloud(cloud_input, cloud_trans3, trans_matrix);
    // // 创建可视化对象
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    // viewer->setBackgroundColor(1.0, 1.0, 1.0);  // 设置背景颜色
    // // 添加点云数据
    // viewer->addPointCloud<pcl::PointXYZ>(cloud_source.makeShared(), "cloud1");
    // viewer->addPointCloud<pcl::PointXYZ>(cloud_trans2.makeShared(), "cloud2");
    // viewer->addPointCloud<pcl::PointXYZ>(cloud_trans3.makeShared(), "cloud3");
    // // 设置点云颜色 原始点云红色  预测点云绿色  配准点云蓝色
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud2");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud3");
    // // 设置点云大小
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud2");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud3");
    // // 运行可视化窗口
    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    //     usleep(200);
    // }

    *pose_estimate = cartographer::transform::Project2D(ToRigid3d(trans_matrix));
  } else {
    *pose_estimate = predict_pose;
  }


}
