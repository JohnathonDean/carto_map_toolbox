#include "carto_map/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "carto_map/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "carto_map/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include <opencv2/opencv.hpp>

class SubmapMatcher
{
public:
    SubmapMatcher(const RealTimeCorrelativeScanMatcherParam& correlative_scan_matcher_options,
                const FastCorrelativeScanMatcherParam& fast_correlative_scan_matcher_options,
                const CeresScanMatcherParam& ceres_scan_matcher_options);
    ~SubmapMatcher();

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

    cartographer::sensor::PointCloud GetPointcloudFromGrid(const cartographer::mapping::Grid2D& grid);

    void SaveImageFromGrid(const cartographer::mapping::Grid2D& grid, const std::string& file_name) {
        cv::Mat image(1000, 1000, CV_8U, cv::Scalar(255));

        Eigen::Array2i offset;
        cartographer::mapping::CellLimits cell_limits;
        grid.ComputeCroppedLimits(&offset, &cell_limits);
        if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
            LOG(WARNING) << "Empty grid!";
            return;
        }

        for (const Eigen::Array2i& xy_index : cartographer::mapping::XYIndexRangeIterator(cell_limits)) {
            const Eigen::Array2i index = xy_index + offset;
            if (!grid.IsKnown(index)) continue;
            // std::cout << grid.GetCorrespondenceCost(index) << "/" << grid.GetMaxCorrespondenceCost() << ";";
            if(1.f - grid.GetCorrespondenceCost(index) > 0.5) {
                int x = static_cast<int>(500 + index.x());
                int y = static_cast<int>(500 + index.y());
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    image.at<uchar>(y, x) = 0;  // 设置像素值为黑色
                }
            }
        }
        cv::imwrite(file_name, image);
    }

    void SaveImageFromMatchGrid(const cartographer::mapping::Grid2D& input_grid,
                                const cartographer::mapping::Grid2D& source_grid,
                                const cartographer::transform::Rigid2d& predict_pose,
                                const std::string& file_name) {
        cv::Mat image(6000, 6000, CV_8UC3, cv::Scalar(255, 255, 255));

        cartographer::sensor::PointCloud input = GetPointcloudFromGrid(input_grid);
        cartographer::sensor::PointCloud source = GetPointcloudFromGrid(source_grid);
        // 遍历点云数据并绘制到图像上
        for (const auto& point : input.points()) {
            int x = static_cast<int>(point.position.x() * 100 + image.cols / 2); // 假设放大了 100 倍，并将原点平移到图像中心
            int y = static_cast<int>(point.position.y() * 100 + image.rows / 2);
            cv::circle(image, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1); // -1 表示实心圆 红色
        }
        for (const auto& point : source.points()) {
            int x = static_cast<int>(point.position.x() * 100 + image.cols / 2); // 假设放大了 100 倍，并将原点平移到图像中心
            int y = static_cast<int>(point.position.y() * 100 + image.rows / 2);
            cv::circle(image, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1); // -1 表示实心圆 蓝色
        }

        cv::imwrite(file_name, image);
    }


private:
    RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;
    CeresScanMatcher2D ceres_scan_matcher_;

    const FastCorrelativeScanMatcherParam fast_correlative_scan_matcher_param;
    std::shared_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher;

};

SubmapMatcher::SubmapMatcher(const RealTimeCorrelativeScanMatcherParam& correlative_scan_matcher_options,
                const FastCorrelativeScanMatcherParam& fast_correlative_scan_matcher_options,
                const CeresScanMatcherParam& ceres_scan_matcher_options)
    : real_time_correlative_scan_matcher_(correlative_scan_matcher_options),
      ceres_scan_matcher_(ceres_scan_matcher_options),
      fast_correlative_scan_matcher_param(fast_correlative_scan_matcher_options)
{

}

SubmapMatcher::~SubmapMatcher()
{
}


cartographer::sensor::PointCloud SubmapMatcher::GetPointcloudFromGrid(const cartographer::mapping::Grid2D& grid) {
    cartographer::sensor::PointCloud pointcloud_res;
    Eigen::Array2i offset;
    cartographer::mapping::CellLimits cell_limits;
    grid.ComputeCroppedLimits(&offset, &cell_limits);
    if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
        LOG(WARNING) << "Empty grid!";
        return pointcloud_res;
    }

    for (const Eigen::Array2i& xy_index : cartographer::mapping::XYIndexRangeIterator(cell_limits)) {
        const Eigen::Array2i index = xy_index + offset;
        if (!grid.IsKnown(index)) continue;
        if(1.f - grid.GetCorrespondenceCost(index) > 0.5) {
            double point_x = grid.limits().max().x() - grid.limits().resolution() * (index.y() + 0.5);
            double point_y = grid.limits().max().y() - grid.limits().resolution() * (index.x() + 0.5);
            Eigen::Vector3f point_pose(point_x, point_y, 0);
            pointcloud_res.push_back({point_pose});
        }
    }
    return pointcloud_res;
}

void SubmapMatcher::MatchCSM(const cartographer::mapping::Grid2D& input_grid,
                            const cartographer::mapping::Grid2D& source_grid,
                            const cartographer::transform::Rigid2d& predict_pose,
                            cartographer::transform::Rigid2d* pose_estimate) {
    cartographer::sensor::PointCloud input_point_cloud = GetPointcloudFromGrid(input_grid);

    fast_correlative_scan_matcher = std::make_shared<FastCorrelativeScanMatcher2D>(
                        source_grid, fast_correlative_scan_matcher_param);
    cartographer::transform::Rigid2d initial_ceres_pose = predict_pose;
    float score = 0.;
    fast_correlative_scan_matcher->Match(predict_pose, input_point_cloud, 0.2, &score, &initial_ceres_pose);
    // fast_correlative_scan_matcher->MatchFullSubmap(input_point_cloud, 0.2, &score, &initial_ceres_pose);
    LOG(INFO) << "predict_pose" << predict_pose;
    LOG(INFO) << "fast_correlative_scan_matcher" << initial_ceres_pose
            << "   score: " << score;

    cartographer::transform::Rigid2d pose_observation = predict_pose;
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(predict_pose.translation(), initial_ceres_pose,
                            input_point_cloud,
                            source_grid, &pose_observation,
                            &summary);
    LOG(INFO) << "ceres_scan_matcher" << pose_observation;

    *pose_estimate = pose_observation;
}

void SubmapMatcher::MatchFullCSM(const cartographer::mapping::Grid2D& input_grid,
                                const cartographer::mapping::Grid2D& source_grid,
                                const cartographer::transform::Rigid2d& predict_pose,
                                cartographer::transform::Rigid2d* pose_estimate) {
    cartographer::sensor::PointCloud input_point_cloud = GetPointcloudFromGrid(input_grid);

    fast_correlative_scan_matcher = std::make_shared<FastCorrelativeScanMatcher2D>(
                        source_grid, fast_correlative_scan_matcher_param);
    cartographer::transform::Rigid2d initial_ceres_pose = predict_pose;
    float score = 0.;
    // fast_correlative_scan_matcher->Match(predict_pose, input_point_cloud, 0.2, &score, &initial_ceres_pose);
    fast_correlative_scan_matcher->MatchFullSubmap(input_point_cloud, 0.2, &score, &initial_ceres_pose);
    LOG(INFO) << "predict_pose" << predict_pose;
    LOG(INFO) << "fast_correlative_scan_matcher" << initial_ceres_pose
            << "   score: " << score;

    cartographer::transform::Rigid2d pose_observation = predict_pose;
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(predict_pose.translation(), initial_ceres_pose,
                            input_point_cloud,
                            source_grid, &pose_observation,
                            &summary);
    LOG(INFO) << "ceres_scan_matcher" << pose_observation;

    *pose_estimate = pose_observation;
}


void SubmapMatcher::MatchRT(const cartographer::mapping::Grid2D& input_grid,
                            const cartographer::mapping::Grid2D& source_grid,
                            const cartographer::transform::Rigid2d& predict_pose,
                            cartographer::transform::Rigid2d* pose_estimate) {
    cartographer::sensor::PointCloud input_point_cloud = GetPointcloudFromGrid(input_grid);

    double angle_scaling_factor_ = 1.0;
    cartographer::transform::Rigid2d initial_ceres_pose = predict_pose;
    const double score = real_time_correlative_scan_matcher_.Match(
        predict_pose, input_point_cloud,
        source_grid, &initial_ceres_pose, angle_scaling_factor_);
    LOG(INFO) << "predict_pose" << predict_pose;
    LOG(INFO) << "real_time_correlative_scan_matcher" << initial_ceres_pose
            << "   score: " << score;

    cartographer::transform::Rigid2d pose_observation = predict_pose;
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(predict_pose.translation(), initial_ceres_pose,
                            input_point_cloud,
                            source_grid, &pose_observation,
                            &summary);
    LOG(INFO) << "ceres_scan_matcher" << pose_observation;

    *pose_estimate = pose_observation;
}



