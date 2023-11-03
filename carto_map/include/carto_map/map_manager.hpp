#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/mapping/pose_graph_interface.h>

#include "carto_map/LoadMap.h"
#include "carto_map/OptimizeSubmap.h"
#include "carto_map/RemoveSubmap.h"
#include "carto_map/RemoveTrajectory.h"
#include "carto_map/SaveMap.h"
#include "carto_map/map_writer.hpp"
#include "carto_map/msg_conversion.hpp"
#include "cartographer/io/color.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "std_srvs/Trigger.h"
#include "visualization_msgs/MarkerArray.h"

using ::cartographer::transform::Rigid3d;

::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = absl::StrCat("Trajectory ", trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = 0.07;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

struct MapInfo {
  std::string map_id = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
  float resolution = 0.05;
  uint32_t row_size = 0;
  uint32_t col_size = 0;
  int32_t origin_x = 0;
  int32_t origin_y = 0;
  float occupied_threshold = 0;
  float empty_threshold = 0;
  std::vector<cartographer::mapping::SubmapId> hidden_submap_ids;
  std::vector<int> hidden_trajectory_ids;
};

class MapManager {
 public:
  MapManager(const std::string& map_frame);
  ~MapManager();

  cartographer::mapping::PoseGraphMap* pose_graph() {
    return pose_graph_.get();
  }

  bool LoadMap(const std::string& file_name, bool show_disable);
  bool SaveMap(const std::string& file_path);

  cartographer_ros_msgs::SubmapList GetSubmapList();
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetConstraintList();

  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  void HandleOptimizeSubmap(carto_map::OptimizeSubmap::Request& request,
                            carto_map::OptimizeSubmap::Response& response);
  void HandleRemoveSubmap(carto_map::RemoveSubmap::Request& request,
                          carto_map::RemoveSubmap::Response& response);
  void HandleRemoveTrajectory(carto_map::RemoveTrajectory::Request& request,
                              carto_map::RemoveTrajectory::Response& response);
  void ComputeOverlappedSubmaps();

 private:
  /* data */
  bool show_disable_submap;
  MapInfo map_info_;
  std::string map_frame_;
  std::unique_ptr<cartographer::mapping::PoseGraphMap> pose_graph_;

  int trajectory_num_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;

  std::string SubmapToProto(
      const cartographer::mapping::SubmapId& submap_id,
      cartographer::mapping::proto::SubmapQuery::Response* const response);
};

MapManager::MapManager(const std::string& map_frame) : map_frame_(map_frame) {
  pose_graph_ = absl::make_unique<cartographer::mapping::PoseGraphMap>();
}

MapManager::~MapManager() {}

bool MapManager::LoadMap(const std::string& file_name, bool show_disable) {
  show_disable_submap = show_disable;
  std::string pbs_path = file_name + "/" + "grid2d.pbstream";
  LOG(INFO) << "Loading saved pbstream '" << pbs_path << "'...";
  cartographer::io::ProtoStreamReader stream(pbs_path);

  auto start_time = std::chrono::high_resolution_clock::now();
  pose_graph_->LoadStateFromProto(&stream);
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "LoadMap cost time:" << duration_time;

  std::string map_data_path = file_name + "/" + "meta.json";
  boost::property_tree::ptree map_data;
  try {
    boost::property_tree::read_json(map_data_path, map_data);
  } catch (boost::property_tree::ptree_error& e) {
    LOG(ERROR) << "LoadMapData failed:" << e.what();
  }

  // 读取必需配置
  auto resolution = map_data.get_optional<float>("resolution");
  auto origin_x = map_data.get_optional<float>("origin_x");
  auto origin_y = map_data.get_optional<float>("origin_y");
  auto size_x = map_data.get_optional<uint32_t>("size_x");
  auto size_y = map_data.get_optional<uint32_t>("size_y");
  auto occupied_threshold = map_data.get_optional<float>("occupied_threshold");
  auto empty_threshold = map_data.get_optional<float>("empty_threshold");
  auto map_id = map_data.get_optional<std::string>("id");

  map_info_.resolution = *resolution;
  map_info_.origin_x = *origin_x;
  map_info_.origin_y = *origin_y;
  map_info_.col_size = *size_x;
  map_info_.row_size = *size_y;
  map_info_.occupied_threshold = *occupied_threshold;
  map_info_.empty_threshold = *empty_threshold;
  map_info_.map_id = *map_id;

  auto map_hidden_submaps_in_meta_cfg =
      map_data.get_child_optional("hidden_submaps");
  if (map_hidden_submaps_in_meta_cfg) {
    for (auto& [_, map_hidden_submaps_in_meta_cfg] :
         *map_hidden_submaps_in_meta_cfg) {
      auto trajectory_id =
          map_hidden_submaps_in_meta_cfg.get_optional<int>("trajectory_id");
      if (!trajectory_id) {
        continue;
      } else {
        auto hidden_all_submaps_cfg =
            map_hidden_submaps_in_meta_cfg.get_child_optional("hidden_all");
        auto hidden_all_submaps =
            map_hidden_submaps_in_meta_cfg.get_optional<int>("hidden_all");
        if (hidden_all_submaps_cfg && hidden_all_submaps == 1) {
          map_info_.hidden_trajectory_ids.push_back(int(*trajectory_id));
        } else {
          auto submap_index_in_meta_cfg =
              map_hidden_submaps_in_meta_cfg.get_child_optional("submap_index");
          if (!submap_index_in_meta_cfg) {
            continue;
          } else {
            for (auto& iter : *submap_index_in_meta_cfg) {
              cartographer::mapping::SubmapId submap_id = {0, 0};
              submap_id.submap_index = iter.second.get_value<int>();
              submap_id.trajectory_id = *trajectory_id;
              map_info_.hidden_submap_ids.push_back(submap_id);
            }
          }
        }
      }
    }
  }

  pose_graph_->LoadSubmapList(map_data);

  return true;
}

bool MapManager::SaveMap(const std::string& file_path) {
  std::string file_name = file_path + "/grid2d.pbstream";
  cartographer::io::ProtoStreamWriter writer(file_name);
  LOG(INFO) << "Saved pbstream as '" << file_name << "'";

  auto start_time = std::chrono::high_resolution_clock::now();
  cartographer::io::WriteMapProto(*pose_graph_, &writer);
  // pose_graph_->WriteProto(&writer);
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();
  LOG(INFO) << "WriteMapProto cost time:" << duration_time;

  return (writer.Close());
}

cartographer_ros_msgs::SubmapList MapManager::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = map_frame_;
  for (const auto& submap_id_pose : pose_graph_->GetAllSubmapPoses()) {
    if (std::find(map_info_.hidden_trajectory_ids.begin(),
                  map_info_.hidden_trajectory_ids.end(),
                  submap_id_pose.id.trajectory_id) !=
        map_info_.hidden_trajectory_ids.end()) {
      continue;
    }
    if (std::find(map_info_.hidden_submap_ids.begin(),
                  map_info_.hidden_submap_ids.end(),
                  submap_id_pose.id) != map_info_.hidden_submap_ids.end()) {
      continue;
    }
    cartographer_ros_msgs::SubmapEntry submap_entry;
    submap_entry.is_frozen = false;
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}

visualization_msgs::MarkerArray MapManager::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto node_poses = pose_graph_->GetTrajectoryNodePoses();
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }
  const auto constraints = pose_graph_->constraints();
  for (const auto& constraint : constraints) {
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTER_SUBMAP) {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                                                             .trajectory_id] =
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
      }
    }
  }

  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::Marker marker =
        CreateTrajectoryMarker(trajectory_id, map_frame_);
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    marker.color.a = 1.0;
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);

      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }
  }
  return trajectory_node_list;
}

visualization_msgs::MarkerArray MapManager::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = map_frame_;
  constraint_intra_marker.scale.x = 0.025;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses = pose_graph_->GetTrajectoryNodePoses();
  const auto submap_poses = pose_graph_->GetAllSubmapPoses();
  const auto constraints = pose_graph_->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      } else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}

void MapManager::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  const std::string error = SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }

  response.submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
}

std::string MapManager::SubmapToProto(
    const cartographer::mapping::SubmapId& submap_id,
    cartographer::mapping::proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= pose_graph_->num_trajectory()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(pose_graph_->num_trajectory()) + " trajectories.";
  }

  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

void MapManager::HandleOptimizeSubmap(
    carto_map::OptimizeSubmap::Request& request,
    carto_map::OptimizeSubmap::Response& response) {
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  std::array<double, 3> input_pose = {request.x, request.y, request.theta};
  pose_graph_->OptimizeSubmapPose(submap_id, input_pose);
}

void MapManager::HandleRemoveSubmap(
    carto_map::RemoveSubmap::Request& request,
    carto_map::RemoveSubmap::Response& response) {
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  pose_graph_->RemoveSubmap(submap_id);
}

void MapManager::HandleRemoveTrajectory(
    carto_map::RemoveTrajectory::Request& request,
    carto_map::RemoveTrajectory::Response& response) {
  pose_graph_->RemoveTrajectory(request.trajectory_id);
}

void MapManager::ComputeOverlappedSubmaps() {
  pose_graph_->ComputeOverlappedSubmaps();
}
