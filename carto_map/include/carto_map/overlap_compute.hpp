#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

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

  OccupancyGrid2D(const boost::property_tree::ptree& submap_data) {
    resolution_ = submap_data.get<double>("resolution");
    max_x_      = submap_data.get<double>("max_x");
    max_y_      = submap_data.get<double>("max_y");
    num_y_      = submap_data.get<int>("num_y");
    num_x_      = submap_data.get<int>("num_x");
    auto global_pose_data = submap_data.get_child("global_pose");
    double x     = global_pose_data.get<double>("x");
    double y     = global_pose_data.get<double>("y");
    double angle = global_pose_data.get<double>("angle");
    global_pose_ = transform::Embed3D(transform::Rigid2d({x, y}, angle));
    auto cell_index_cfg = submap_data.get_child("known_cell_index");
    for (auto& cell_index : cell_index_cfg) {
      known_cell_index_.push_back(cell_index.second.get_value<int>());
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

  boost::property_tree::ptree ToJsonValue() {
    boost::property_tree::ptree submap_data;

    submap_data.put("resolution", resolution_);
    submap_data.put("max_x",      max_x_);
    submap_data.put("max_y",      max_y_);
    submap_data.put("num_x",      num_x_);
    submap_data.put("num_y",      num_y_);
    transform::Rigid2d global_pose_2d = transform::Project2D(global_pose_);
    boost::property_tree::ptree global_pose_data;
    double x = global_pose_2d.translation().x();
    double y = global_pose_2d.translation().y();
    double angle = global_pose_2d.rotation().angle();
    global_pose_data.put("x", x);
    global_pose_data.put("y", y);
    global_pose_data.put("angle", angle);
    submap_data.put_child("global_pose", global_pose_data);
    boost::property_tree::ptree cell_index_data;
    for (const auto& index : known_cell_index_) {
      // cell_index_data.push_back(std::make_pair("", index));
      cell_index_data.add("", index);
    }
    submap_data.put_child("known_cell_index", cell_index_data);

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

class OverlappingSubmapsCompute2D {
 public:
  OverlappingSubmapsCompute2D(const double map_resolution) {
    low_resolution_ = map_resolution;
  }
  ~OverlappingSubmapsCompute2D() {}

  void LoadSubmapList(const boost::property_tree::ptree& submap_data);

  void UpdateSubmapList(
      const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data);

  std::set<SubmapId> GenerateGlobalCoverageGrid2D(
      GlobalCoverageGrid2D* coverage_grid);

  void ComputeAllSubmapsOverlap(
      const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data);

  boost::property_tree::ptree GetSubmapListToJson();

 private:
  double low_resolution_;

  std::map<SubmapId, std::shared_ptr<OccupancyGrid2D>> submaps_grid_;
};

void OverlappingSubmapsCompute2D::UpdateSubmapList(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data) {
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

boost::property_tree::ptree OverlappingSubmapsCompute2D::GetSubmapListToJson() {
  boost::property_tree::ptree submap_list;
  for (const auto& submap : submaps_grid_) {
    boost::property_tree::ptree submap_data = submap.second->ToJsonValue();
    submap_data.put("trajectory_id", submap.first.trajectory_id);
    submap_data.put("submap_index", submap.first.submap_index);
    submap_list.push_back(std::make_pair("", submap_data));
  }
  return submap_list;
}

void OverlappingSubmapsCompute2D::LoadSubmapList(const boost::property_tree::ptree& submap_data) {
  auto submap_list_cfg = submap_data.get_child_optional("submap_list");
  if(!submap_list_cfg)
    return;
  for (auto& submap : *submap_list_cfg) {
    int trajectory_id_tem = *submap.second.get_optional<int>("trajectory_id");
    int submap_index_tem = *submap.second.get_optional<int>("submap_index");
    SubmapId submap_id{trajectory_id_tem, submap_index_tem};
    submaps_grid_[submap_id] = std::make_shared<OccupancyGrid2D>(submap.second);
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

void OverlappingSubmapsCompute2D::ComputeAllSubmapsOverlap(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data) {
  UpdateSubmapList(submap_data);

  Eigen::Vector2d grid_offset(0.0, 0.0);
  GlobalCoverageGrid2D coverage_grid(grid_offset, low_resolution_);
  std::set<SubmapId> all_submap_ids =
      GenerateGlobalCoverageGrid2D(&coverage_grid);
  coverage_grid.SaveImage("/home/Pictures/coverage_grid.png");

  std::ofstream out_file("/home/dean/Pictures/submap_list111.json",
                         std::ios::out | std::ios::binary);
  boost::property_tree::json_parser::write_json(out_file, GetSubmapListToJson());


  // std::string submaps_data = GetSubmapListToJson().dump();
  // std::ofstream out_file("/home/dean/Pictures/submap_list.json",
  //                        std::ios::out | std::ios::binary);
  // out_file.write(submaps_data.data(), submaps_data.size());
  // out_file.close();

  int fresh_submaps_count = 4;
  double max_coverage_rate = 0.85;
  auto start_time = std::chrono::high_resolution_clock::now();
  std::stringstream ss_out;
  ss_out << "Find Overlap submap ids:";
  std::map<SubmapId, uint16> submap_to_covered_cells_count;
  for (const auto& cell : coverage_grid.cells()) {
    std::vector<SubmapId> submaps_per_cell(cell.second);
    // 对于每个cell删掉最新的fresh_submaps_count个观测，记录每个观测中submap_id的保留数量
    if (submaps_per_cell.size() > fresh_submaps_count) {
      std::sort(submaps_per_cell.begin(), submaps_per_cell.end());
      submaps_per_cell.erase(submaps_per_cell.end() - fresh_submaps_count,
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
    ss_out << "\n"
           << id_to_cells_count.first << "*" << id_to_cells_count.second << "/"
           << submaps_grid_[id_to_cells_count.first]->known_cell_index_.size()
           << "[" << coverage_rate << "]";
    if (coverage_rate > max_coverage_rate) {
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
}

// compare to old method
// Iterates over every cell in a submap, transforms the center of the cell to
// the global frame and then adds the submap id and the timestamp of the most
// recent range data insertion into the global grid.
std::map<SubmapId, int> AddSubmapsToGlobalCoverageGrid2D(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    const std::vector<SubmapId>& input_submaps,
    GlobalCoverageGrid2D* coverage_grid) {
  auto start_time = std::chrono::high_resolution_clock::now();
  LOG(INFO) << "Start AddSubmapsToGlobalCoverageGrid2D";

  std::map<SubmapId, int> coverage_submap_ids;
  for (const auto& submap_id : input_submaps) {
    auto start_time_per = std::chrono::high_resolution_clock::now();
    const Grid2D& grid = *std::static_pointer_cast<const Submap2D>(
                              submap_data.find(submap_id)->data.submap)
                              ->grid();
    Eigen::Array2i offset;
    CellLimits cell_limits;
    grid.ComputeCroppedLimits(&offset, &cell_limits);
    const transform::Rigid3d& global_frame_from_submap_frame =
        submap_data.find(submap_id)->data.pose;
    const transform::Rigid3d submap_frame_from_local_frame =
        submap_data.find(submap_id)->data.submap->local_pose().inverse();
    const transform::Rigid3d temp_trans =
        global_frame_from_submap_frame * submap_frame_from_local_frame;
    int known_cell_count = 0;
    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
      const Eigen::Array2i index = xy_index + offset;
      if (!grid.IsKnown(index)) continue;
      known_cell_count++;
      const transform::Rigid3d center_of_cell_in_local_frame =
          transform::Rigid3d::Translation(Eigen::Vector3d(
              grid.limits().max().x() -
                  grid.limits().resolution() * (index.y() + 0.5),
              grid.limits().max().y() -
                  grid.limits().resolution() * (index.x() + 0.5),
              0));

      const transform::Rigid2d center_of_cell_in_global_frame =
          transform::Project2D(temp_trans * center_of_cell_in_local_frame);
      coverage_grid->AddPoint(center_of_cell_in_global_frame.translation(),
                              submap_id);
    }
    coverage_submap_ids[submap_id] = known_cell_count;
    auto end_time_per = std::chrono::high_resolution_clock::now();
    auto duration_time_per =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time_per -
                                                              start_time_per)
            .count();
    LOG(INFO) << "AddSubmaps " << submap_id
              << "To SubmapCoverageGrid2D cost time:" << duration_time_per;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "AddSubmapsToSubmapCoverageGrid2D total cost time:"
            << duration_time;

  return coverage_submap_ids;
}

// Returns IDs of submaps that have less than 'min_covered_cells_count' cells
// not overlapped by at least 'fresh_submaps_count' submaps.
std::vector<SubmapId> FindSubmapOverlapped(
    const GlobalCoverageGrid2D& coverage_grid,
    std::map<SubmapId, int> submap_cells, uint16 fresh_submaps_count,
    double max_coverage_rate) {
  auto start_time = std::chrono::high_resolution_clock::now();
  LOG(INFO) << "Start FindSubmapOverlapped";
  std::stringstream ss_out;
  ss_out << "Find trimming submap ids:";

  std::map<SubmapId, uint16> submap_to_covered_cells_count;
  for (const auto& cell : coverage_grid.cells()) {
    std::vector<SubmapId> submaps_per_cell(cell.second);
    // 对于每个cell删掉最新的fresh_submaps_count个观测，记录每个观测中submap_id的保留数量
    if (submaps_per_cell.size() > fresh_submaps_count) {
      std::sort(submaps_per_cell.begin(), submaps_per_cell.end());
      submaps_per_cell.erase(submaps_per_cell.end() - fresh_submaps_count,
                             submaps_per_cell.end());
      for (const SubmapId& submap : submaps_per_cell) {
        ++submap_to_covered_cells_count[submap];
      }
    }
  }
  std::vector<SubmapId> submap_ids_to_remove;
  for (const auto& id_to_cells_count : submap_to_covered_cells_count) {
    double coverage_rate =
        id_to_cells_count.second * 1.0 / submap_cells[id_to_cells_count.first];
    ss_out << "\n"
           << id_to_cells_count.first << "*" << id_to_cells_count.second << "/"
           << submap_cells[id_to_cells_count.first] << "[" << coverage_rate
           << "]";
    if (coverage_rate > max_coverage_rate) {
      submap_ids_to_remove.push_back(id_to_cells_count.first);
      ss_out << "====";
    }
  }
  LOG(INFO) << ss_out.str();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "FindSubmapIdsToTrim cost time:" << duration_time;

  DCHECK(
      std::is_sorted(submap_ids_to_remove.begin(), submap_ids_to_remove.end()));
  return submap_ids_to_remove;
}

void ComputeAllSubmapsOverlap(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data) {
  LOG(INFO) << "Start ComputeAllSubmapsOverlap";

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<SubmapId> input_submap_ids;
  for (const auto& submap : submap_data) {
    input_submap_ids.push_back(submap.id);
  }

  Eigen::Vector2d grid_offset(0.0, 0.0);
  GlobalCoverageGrid2D coverage_grid(grid_offset, 0.05);

  const std::map<SubmapId, int> all_submap_ids =
      AddSubmapsToGlobalCoverageGrid2D(submap_data, input_submap_ids,
                                       &coverage_grid);
  const std::vector<SubmapId> submap_ids_to_remove =
      FindSubmapOverlapped(coverage_grid, all_submap_ids, 4, 0.7);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "ComputeAllSubmapsOverlap cost time:" << duration_time;
}

}  // namespace mapping
}  // namespace cartographer
