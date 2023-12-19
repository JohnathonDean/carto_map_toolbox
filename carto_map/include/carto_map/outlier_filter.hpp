#include "cartographer/sensor/point_cloud.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>

cartographer::sensor::PointCloud FromPCLPointCloud(const pcl::PointCloud<pcl::PointXYZ>& point_cloud) {
    cartographer::sensor::PointCloud res_points;
    for (const auto& pcl_point : point_cloud.points) {
        res_points.push_back({Eigen::Vector3f(pcl_point.x, pcl_point.y, pcl_point.z)});
    }
    return res_points;
}

pcl::PointCloud<pcl::PointXYZ> ToPCLPointCloud(const cartographer::sensor::PointCloud& point_cloud) {
    pcl::PointCloud<pcl::PointXYZ> res_points;
    for (std::size_t i = 0; i < point_cloud.size(); ++i) {
        res_points.push_back(pcl::PointXYZ(point_cloud[i].position.x(),
                                            point_cloud[i].position.y(),
                                            point_cloud[i].position.z()));
    }
    return res_points;
}

cartographer::sensor::PointCloud StatisticalOutlierFilterPCL(const cartographer::sensor::PointCloud& point_cloud,
    const int mean_k, const double std_mul){
    pcl::PointCloud<pcl::PointXYZ> input = ToPCLPointCloud(point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);//滤波后的结果

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input.makeShared());    //输入点云
    sor.setMeanK(mean_k);            //设置领域点的个数
    sor.setStddevMulThresh(std_mul); //设置离群点的阈值
    sor.filter(*cloud_filtered); //滤波后的结果

    cartographer::sensor::PointCloud res_points;
    res_points = FromPCLPointCloud(*cloud_filtered);
    LOG(INFO) << "StatisticalOutlierFilter input size:" << point_cloud.size()
                << "; output size:" << res_points.size();
    return res_points;
}


cartographer::sensor::PointCloud StatisticalOutlierFilter(const cartographer::sensor::PointCloud& point_cloud,
    const int mean_k, const double std_mul){
    cartographer::sensor::PointCloud res_points;
    std::vector<float> distances;
    for (std::size_t i = 0; i < point_cloud.size(); ++i) {
        // 计算每个点与其它点的距离
        for (std::size_t j = 0; j < point_cloud.size(); ++j) {
            if (i != j) {
                float dis = (point_cloud[i].position - point_cloud[j].position).norm();
                distances.push_back(dis);
            }
        }
        // 对距离进行排序
        std::sort(distances.begin(), distances.end());
        // 计算均值和标准差
        float mean = 0.0f;
        for (int k = 0; k < mean_k; ++k) {
            mean += distances[k];
        }
        mean /= mean_k;
        float stddev = 0.0f;
        for (int k = 0; k < mean_k; ++k) {
            stddev += std::pow(distances[k] - mean, 2);
        }
        stddev = std::sqrt(stddev / mean_k);

        // 移除离群点
        if (distances[mean_k - 1] > mean + std_mul * stddev) {
            // point_cloud[i].position = (0.0f, 0.0f, 0.0f); // 标记为离群点
        } else {
            res_points.push_back(point_cloud[i]);
        }
    }
    LOG(INFO) << "StatisticalOutlierFilter input size:" << point_cloud.size()
            << "; output size:" << res_points.size();
    return res_points;
}


