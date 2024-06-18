#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "cartographer/sensor/point_cloud.h"


namespace icp_matcher {

using PointCloud = std::vector<Eigen::Vector3f>;

PointCloud fromPclPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input) {
    PointCloud res;
    res.reserve(input.size());
    for(const auto& point : input.points) {
        res.push_back({point.x, point.y, point.z});
    }
    return res;
}

pcl::PointCloud<pcl::PointXYZ> toPclPointcloud(const PointCloud& input) {
    pcl::PointCloud<pcl::PointXYZ> res;
    res.reserve(input.size());
    for(const auto& point : input) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        res.points.push_back(pcl_point);
    }
    return res;
}


// 计算质心
Eigen::Vector3f computeCentroid(const PointCloud& cloud) {
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& point : cloud) {
        centroid += point;
    }
    centroid /= cloud.size();
    return centroid;
}

// 寻找最近点,使用暴力搜索 TODO:使用KD-Tree在目标点云中找到每个源点的最近点
std::vector<int> findNearestNeighbors(const PointCloud& source, const PointCloud& target) {
    std::vector<int> indices;
    for (const auto& point : source) {
        float min_dist = std::numeric_limits<float>::max();
        int min_index = -1;
        for (size_t i = 0; i < target.size(); ++i) {
            float dist = (point - target[i]).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                min_index = i;
            }
        }
        indices.push_back(min_index);
    }
    return indices;
}

// 计算最佳刚体变换
Eigen::Matrix4f computeTransformation(const PointCloud& source, const PointCloud& target, const std::vector<int>& indices) {
    Eigen::Vector3f centroid_src = computeCentroid(source);
    Eigen::Vector3f centroid_tgt = computeCentroid(target);

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, 3);
    for (size_t i = 0; i < source.size(); ++i) {
        Eigen::Vector3f p = source[i] - centroid_src;
        Eigen::Vector3f q = target[indices[i]] - centroid_tgt;
        H += p * q.transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
        R.col(2) *= -1;
    }

    Eigen::Vector3f t = centroid_tgt - R * centroid_src;

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;
    return transformation;
}

// 应用变换到点云
PointCloud transformPointCloud(const PointCloud& cloud, const Eigen::Matrix4f& transform) {
    PointCloud transformed_cloud;
    for (const auto& point : cloud) {
        Eigen::Vector4f p(point.x(), point.y(), point.z(), 1.0f);
        Eigen::Vector4f p_transformed = transform * p;
        transformed_cloud.push_back(p_transformed.head<3>());
    }
    return transformed_cloud;
}

void MatchICP(const PointCloud& cloud_input, const PointCloud& cloud_source,
            const Eigen::Matrix4f& predict_pose, Eigen::Matrix4f* pose_estimate,
            int max_iterations = 50, float tolerance = 1e-6) {
    Eigen::Matrix4f current_pose = predict_pose;
    PointCloud source_transformed = transformPointCloud(cloud_source, current_pose);
    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<int> nearest_indices = findNearestNeighbors(source_transformed, cloud_input);
        Eigen::Matrix4f transformation = computeTransformation(source_transformed, cloud_input, nearest_indices);
        current_pose = transformation * current_pose;
        source_transformed = transformPointCloud(cloud_source, current_pose);
        if (transformation.block<3, 1>(0, 3).norm() < tolerance) {
            break;
        }
    }
    *pose_estimate = current_pose;
}





}



