#include <fstream>
#include <thread>
#include <opencv2/opencv.hpp>

#include "json/json.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"

namespace cartographer {
namespace mapping {

struct Array2iComparator {
  bool operator()(const Eigen::Array2i& a, const Eigen::Array2i& b) const {
    // 定义比较规则，例如按 x 坐标升序排序
    if (a.x() < b.x()) {
      return true;
    }
    if (a.x() > b.x()) {
      return false;
    }
    return a.y() < b.y();
  }
};

class GlobalCoverageGrid2D {
 public:
  // Aliases for documentation only (no type-safety).
  using CellId = std::pair<int64 /* x cells */, int64 /* y cells */>;
  using StoredType = std::vector<SubmapId>;

  GlobalCoverageGrid2D(const Eigen::Vector2d& map_offset,
                       const double map_resolution)
      : offset_(map_offset), resolution_(map_resolution) {}

  void AddPoint(const Eigen::Vector2d& point, const SubmapId& submap_id) {
    CellId cell_id{common::RoundToInt64((point(0) - offset_(0)) / resolution_),
                   common::RoundToInt64((point(1) - offset_(1)) / resolution_)};
    cells_[cell_id].emplace_back(submap_id);
  }

  const std::map<CellId, StoredType>& cells() const { return cells_; }
  double resolution() const { return resolution_; }
  void SaveImage(const std::string& file_name) {
    cv::Mat image(800, 800, CV_8U, cv::Scalar(255));
    // std::stringstream ss_out;
    // ss_out << "SaveImage from indexs:";
    for (const auto& cell : cells_) {
      int x = static_cast<int>(250 + cell.first.first);
      int y = static_cast<int>(250 - cell.first.second);
      // ss_out << "[" << x << "," << y << "]";
      if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
        image.at<uchar>(y, x) = 0;  // 设置像素值为黑色
      }
    }
    // LOG(INFO) << ss_out.str();
    cv::imwrite(file_name, image);
  }

 private:
  Eigen::Vector2d offset_;
  double resolution_;
  std::map<CellId, StoredType> cells_;
};

class OccupancyGrid2D {
 public:
  OccupancyGrid2D(const PoseGraphInterface::SubmapData& submap_data,
                  const double lower_resolution) {
    resolution_ = lower_resolution;
    const Grid2D& tar_grid =
        *std::static_pointer_cast<const Submap2D>(submap_data.submap)->grid();
    max_x_ = tar_grid.limits().max().x();
    max_y_ = tar_grid.limits().max().y();
    num_y_ = std::ceil(tar_grid.limits().cell_limits().num_y_cells *
                       tar_grid.limits().resolution() / resolution_);
    num_x_ = std::ceil(tar_grid.limits().cell_limits().num_x_cells *
                       tar_grid.limits().resolution() / resolution_);
    global_pose_ =
        submap_data.pose * submap_data.submap->local_pose().inverse();

    Eigen::Array2i tar_offset;
    CellLimits tar_cell_limits;
    tar_grid.ComputeCroppedLimits(&tar_offset, &tar_cell_limits);
    std::set<Eigen::Array2i, Array2iComparator> known_cells;
    for (const Eigen::Array2i& xy_index :
         XYIndexRangeIterator(tar_cell_limits)) {
      const Eigen::Array2i index = xy_index + tar_offset;
      if (tar_grid.IsKnown(index)) {
        const Eigen::Vector2f local_coordinates(
            tar_grid.limits().max().x() -
                tar_grid.limits().resolution() * (index.y() + 0.5),
            tar_grid.limits().max().y() -
                tar_grid.limits().resolution() * (index.x() + 0.5));
        Eigen::Array2i index_trans = GetCellIndex(local_coordinates);
        known_cells.insert(index_trans);
        // LOG(INFO) << "Get Grid2D in size " << max_x_ << " * " << max_y_
        //           << " in resolution:" << tar_grid.limits().resolution()
        //           << " to lower_resolution:" << resolution_
        //           << " previous cell index " << index.x() << "*" << index.y()
        //           << " local_coordinates " << local_coordinates.x() << "*" <<
        //           local_coordinates.y()
        //           << " trans cell index "  << index_trans.x() << "*" <<
        //           index_trans.y();
      }
    }
    for (const auto& index : known_cells) {
      known_cell_index_.push_back(ToFlatIndex(index));
    }
  }

  OccupancyGrid2D(const Json::Value& submap_data) {
    resolution_  = submap_data["resolution"].asDouble();
    max_x_       = submap_data["max_x"].asDouble();
    max_y_       = submap_data["max_y"].asDouble();
    num_y_       = submap_data["num_y"].asInt();
    num_x_       = submap_data["num_x"].asInt();
    double x     = submap_data["global_pose"]["x"].asDouble();
    double y     = submap_data["global_pose"]["y"].asDouble();
    double angle = submap_data["global_pose"]["angle"].asDouble();
    global_pose_ = transform::Embed3D(transform::Rigid2d({x, y}, angle));
    for (auto& cell_index : submap_data["known_cell_index"]) {
      known_cell_index_.push_back(cell_index.asInt());
    }
  }

  ~OccupancyGrid2D() {}

  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    return Eigen::Array2i(
        common::RoundToInt((max_y_ - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_x_ - point.x()) / resolution_ - 0.5));
  }

  int ToFlatIndex(const Eigen::Array2i& cell_index) const {
    // LOG(INFO) << "ToFlatIndex in size " << num_x_ << " * " << num_y_
    //           << " get x:" << cell_index.x() << " y:" << cell_index.y();
    CHECK((Eigen::Array2i(0, 0) <= cell_index).all() &&
          (cell_index <= Eigen::Array2i(num_x_, num_y_)).all())
        << cell_index;
    return num_x_ * cell_index.y() + cell_index.x();
  }

  Eigen::Array2i ToXYIndex(const int index) const {
    // LOG(INFO) << "ToXYIndex in size " << num_x_ << " * " << num_y_ << "="
    //           << num_x_ * num_y_ << " get index:" << index;
    CHECK(index <= num_x_ * num_y_) << index;
    return Eigen::Array2i(index % num_x_, index / num_x_);
  }

  Json::Value ToJsonValue() {
    Json::Value submap_data;
    submap_data["resolution"] = resolution_;
    submap_data["max_x"] = max_x_;
    submap_data["max_y"] = max_y_;
    submap_data["num_x"] = num_x_;
    submap_data["num_y"] = num_y_;
    transform::Rigid2d global_pose_2d = transform::Project2D(global_pose_);
    submap_data["global_pose"]["x"] = static_cast<double>(global_pose_2d.translation().x());
    submap_data["global_pose"]["y"] = static_cast<double>(global_pose_2d.translation().y());
    submap_data["global_pose"]["angle"] = static_cast<double>(global_pose_2d.rotation().angle());
    for (const auto& index : known_cell_index_) {
      submap_data["known_cell_index"].append(index);
    }
    return submap_data;
  }

  double resolution_;
  double max_x_;
  double max_y_;
  std::vector<int> known_cell_index_;

  int num_x_;
  int num_y_;
  transform::Rigid3d global_pose_ = transform::Rigid3d::Identity();
};

struct OverlappingSubmapsComputeParam{
  int fresh_submaps_count;
  double min_covered_area;
  double max_loss_area;
  double low_resolution;
};

class OverlappingSubmapsCompute2D {
 public:
  OverlappingSubmapsCompute2D(const OverlappingSubmapsComputeParam& param) {
    fresh_submaps_count_ = param.fresh_submaps_count;
    min_covered_area_ = param.min_covered_area;
    max_loss_area_ = param.max_loss_area;
    low_resolution_ = param.low_resolution;
  }
  ~OverlappingSubmapsCompute2D() {}

  void LoadSubmapList(const Json::Value& submap_data);

  void UpdateSubmapList(
      const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data);

  std::set<SubmapId> GenerateGlobalCoverageGrid2D(
      GlobalCoverageGrid2D* coverage_grid);

  std::vector<SubmapId> ComputeAllSubmapsOverlap(
      const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data);

  Json::Value GetSubmapListToJson();

 private:
  absl::Mutex submaps_list_mutex_;

  int fresh_submaps_count_;
  double min_covered_area_;
  double max_loss_area_;
  double low_resolution_;

  std::map<SubmapId, std::shared_ptr<OccupancyGrid2D>> submaps_grid_;
};

void OverlappingSubmapsCompute2D::UpdateSubmapList(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data) {
  absl::MutexLock lock(&submaps_list_mutex_);
  auto start_time = std::chrono::high_resolution_clock::now();
  for (const auto& submap : submap_data) {
    if (submaps_grid_.count(submap.id) == 0) {
      submaps_grid_[submap.id] =
          std::make_shared<OccupancyGrid2D>(submap.data, low_resolution_);
      LOG(INFO) << "AddSubmaps " << submap.id << "To SubmapList";
    }
  }

  std::set<SubmapId> submap_to_remove;
  for (const auto& submap : submaps_grid_) {
    if (!submap_data.Contains(submap.first)) {
      submap_to_remove.insert(submap.first);
    }
  }
  for (const auto& submap_id : submap_to_remove) {
    LOG(INFO) << "RemoveSubmaps " << submap_id << "From SubmapList";
    submaps_grid_.erase(submap_id);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "UpdateSubmapList total cost time:" << duration_time;
}

Json::Value OverlappingSubmapsCompute2D::GetSubmapListToJson() {
  absl::MutexLock lock(&submaps_list_mutex_);
  Json::Value submap_list;
  for (const auto& submap : submaps_grid_) {
    Json::Value submap_data = submap.second->ToJsonValue();
    submap_data["trajectory_id"] = submap.first.trajectory_id;
    submap_data["submap_index"] = submap.first.submap_index;
    submap_list["submap_list"].append(submap_data);
  }
  return submap_list;
}

void OverlappingSubmapsCompute2D::LoadSubmapList(const Json::Value& submap_data) {
  if(!submap_data.isMember("submap_list"))
    return;
  absl::MutexLock lock(&submaps_list_mutex_);
  for (auto& submap : submap_data["submap_list"]) {
    int trajectory_id_tem = submap["trajectory_id"].asInt();
    int submap_index_tem  = submap["submap_index"].asInt();
    SubmapId submap_id{trajectory_id_tem, submap_index_tem};
    submaps_grid_[submap_id] = std::make_shared<OccupancyGrid2D>(submap);
  }
}

std::set<SubmapId> OverlappingSubmapsCompute2D::GenerateGlobalCoverageGrid2D(
    GlobalCoverageGrid2D* coverage_grid) {
  auto start_time = std::chrono::high_resolution_clock::now();

  std::set<SubmapId> submap_ids;
  for (const auto& submap : submaps_grid_) {
    auto start_time_per = std::chrono::high_resolution_clock::now();

    submap_ids.insert(submap.first);
    auto& submap_grid = submap.second;
    for (const auto& cell_index : submap_grid->known_cell_index_) {
      const Eigen::Array2i xy_index = submap_grid->ToXYIndex(cell_index);

      const transform::Rigid3d center_of_cell_in_local_frame =
          transform::Rigid3d::Translation(
              Eigen::Vector3d(submap_grid->max_x_ - submap_grid->resolution_ *
                                                        (xy_index.y() + 0.5),
                              submap_grid->max_y_ - submap_grid->resolution_ *
                                                        (xy_index.x() + 0.5),
                              0));

      const transform::Rigid2d center_of_cell_in_global_frame =
          transform::Project2D(submap_grid->global_pose_ *
                               center_of_cell_in_local_frame);
      coverage_grid->AddPoint(center_of_cell_in_global_frame.translation(),
                              submap.first);
    }

    auto end_time_per = std::chrono::high_resolution_clock::now();
    auto duration_time_per =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time_per -
                                                              start_time_per)
            .count();
    LOG(INFO) << "AddSubmaps " << submap.first
              << "To CoverageGrid2D cost time:" << duration_time_per;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "GenerateGlobalCoverageGrid2D total cost time:" << duration_time;

  return submap_ids;
}

std::vector<SubmapId> OverlappingSubmapsCompute2D::ComputeAllSubmapsOverlap(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data) {
  UpdateSubmapList(submap_data);

  Eigen::Vector2d grid_offset(0.0, 0.0);
  GlobalCoverageGrid2D coverage_grid(grid_offset, low_resolution_);
  std::set<SubmapId> all_submap_ids =
      GenerateGlobalCoverageGrid2D(&coverage_grid);
  // coverage_grid.SaveImage("/home/dean/Pictures/coverage_grid.png");

  // std::ofstream out_file("/home/dean/Pictures/submap_list111.json",
  //                        std::ios::out | std::ios::binary);
	// Json::StyledWriter style_writer;
  // out_file << style_writer.write(GetSubmapListToJson());
  // out_file.close();

  double max_coverage_rate = 0.85;
  auto start_time = std::chrono::high_resolution_clock::now();
  std::stringstream ss_out;
  ss_out << "Find Overlap submap ids:";
  std::map<SubmapId, uint16> submap_to_covered_cells_count;
  std::map<SubmapId, uint16> submap_to_keep_cells_count;
  for (const auto& cell : coverage_grid.cells()) {
    std::vector<SubmapId> submaps_per_cell(cell.second);
    if (submaps_per_cell.size() <= 3) {
      // 对于每个cell观测中submap_id的数量少于3个需要保留，防止删除过度
      for (const SubmapId& submap : submaps_per_cell) {
        ++submap_to_keep_cells_count[submap];
      }
    } else if (submaps_per_cell.size() > fresh_submaps_count_) {
      std::sort(submaps_per_cell.begin(), submaps_per_cell.end());
      // 对于每个cell删掉最新的fresh_submaps_count个观测，记录每个观测中submap_id的保留数量
      submaps_per_cell.erase(submaps_per_cell.end() - fresh_submaps_count_,
                             submaps_per_cell.end());
      for (const SubmapId& submap : submaps_per_cell) {
        ++submap_to_covered_cells_count[submap];
      }
    }
  }
  std::vector<SubmapId> submap_ids_to_remove;
  for (const auto& id_to_cells_count : submap_to_covered_cells_count) {
    double coverage_rate =
        id_to_cells_count.second * 1.0 /
        submaps_grid_[id_to_cells_count.first]->known_cell_index_.size();
    double keep_rate =
            submap_to_keep_cells_count[id_to_cells_count.first] * 1.0 /
        submaps_grid_[id_to_cells_count.first]->known_cell_index_.size();
    ss_out << "\n"
           << id_to_cells_count.first << "*" << id_to_cells_count.second << "/"
           << submaps_grid_[id_to_cells_count.first]->known_cell_index_.size()
           << "[" << coverage_rate << "],[" << keep_rate << "]";
    if (coverage_rate > min_covered_area_ && keep_rate < max_loss_area_) {
      submap_ids_to_remove.push_back(id_to_cells_count.first);
      ss_out << "====";
    }
  }
  LOG(INFO) << ss_out.str();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "FindSubmapIds cost time:" << duration_time;

  return submap_ids_to_remove;
}


}  // namespace mapping
}  // namespace cartographer
