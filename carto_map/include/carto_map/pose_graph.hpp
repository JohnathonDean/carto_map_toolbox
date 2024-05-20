#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>

#include <chrono>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "carto_map/overlap_compute.hpp"
#include "carto_map/pointcloud_map_match.hpp"
#include "carto_map/pose_optimize.hpp"
#include "carto_map/submap_match.hpp"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

using mapping::proto::SerializationHeader;
using mapping::proto::SerializedData;

using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;
using Constraint = ::cartographer::mapping::PoseGraphInterface::Constraint;

class PoseGraphMap {
 public:
  struct InternalSubmapData {
    std::shared_ptr<const Submap> submap;
    std::set<NodeId> node_ids;
  };

 private:
  MapById<SubmapId, InternalSubmapData> submap_data_;
  MapById<SubmapId, transform::Rigid2d> global_submap_poses_2d_;
  MapById<NodeId, TrajectoryNode> trajectory_nodes_;
  std::vector<Constraint> constraints_;
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
  std::map<int, PoseGraphInterface::TrajectoryData> trajectory_data_;
  std::map<std::string, PoseGraphInterface::LandmarkNode> landmark_nodes_;
  std::vector<int> trajectory_id_;
  std::set<int> trajectory_to_delete_;

  // Serialized Map Data
  SerializationHeader header_;
  SerializedData pose_graph_proto_;
  SerializedData trajectory_builder_options_proto_;
  std::vector<SerializedData> submaps_proto_;
  std::vector<SerializedData> trajectory_node_proto_;
  std::vector<SerializedData> all_trajectory_data_proto_;

 public:
  PoseGraphMap() {
    submap_matcher_ = std::make_shared<SubmapMatcher>();
    pcl_matcher_ = std::make_shared<PointcloudMapMatch>();

    OverlappingSubmapsComputeParam param;
    param.fresh_submaps_count = 4;
    param.min_covered_area = 0.95;
    param.max_loss_area = 0.05;
    param.low_resolution = 0.1;
    overlap_computer_ = std::make_shared<OverlappingSubmapsCompute2D>(param);
  }
  ~PoseGraphMap() {}

  PoseGraphMap(const PoseGraphMap&) = delete;
  PoseGraphMap& operator=(const PoseGraphMap&) = delete;

  void LoadStateFromProto(io::ProtoStreamReaderInterface* const reader);

  void LoadSubmapList(const Json::Value& map_data);
  Json::Value GetSubmapListToJson();

  PoseGraphInterface::SubmapData GetSubmapData(const SubmapId& submap_id) const;
  MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData() const;

  MapById<SubmapId, PoseGraphInterface::SubmapPose> GetAllSubmapPoses() const;

  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const;

  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const;

  std::map<std::string, PoseGraphInterface::LandmarkNode> GetLandmarkNodes() const;

  std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const;

  mapping::proto::AllTrajectoryBuilderOptions GetAllTrajectoryBuilderOptions()
      const;

  int num_trajectory() const;

  std::map<int, PoseGraphInterface::TrajectoryData> GetTrajectoryData() const;

  std::vector<Constraint> constraints() const;

  void OptimizeSubmapPose(const SubmapId& submap_id,
                          const std::array<double, 3>& input_pose);

  void ChangeSubmapPose(const SubmapId& submap_id,
                        const std::array<double, 3>& input_pose);

  std::array<double, 3> GetSubmapPose(const SubmapId& submap_id);

  std::vector<SubmapId> ComputeOverlappedSubmaps();

  void RemoveSubmap(const SubmapId& submap_id);
  void RemoveTrajectory(int trajectory_id);

  void DrawSubmap(const SubmapId& submap_id, const std::string& file_name);

  void SubmapPoseOptimization(const SubmapId& input_submap_id,
                              const SubmapId& source_submap_id);

  proto::PoseGraph ToProto() const;
  void WriteProto(io::ProtoStreamWriterInterface* const writer);

  void WriteNodePoseStampToFile(const std::string& file_name);

 private:
  mutable absl::Mutex mutex_;
  ValueConversionTables conversion_tables_;

  std::shared_ptr<OverlappingSubmapsCompute2D> overlap_computer_;
  std::shared_ptr<SubmapMatcher> submap_matcher_;
  std::shared_ptr<PointcloudMapMatch> pcl_matcher_;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto);
  void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                          const proto::Submap& submap);
  void AddNodeFromProto(const transform::Rigid3d& global_pose,
                        const proto::Node& node);
  void AddSerializedConstraints(const std::vector<Constraint>& constraints);
  void SetTrajectoryDataFromProto(const proto::TrajectoryData& data);
  void SetLandmarkPose(const std::string& landmark_id,
                       const transform::Rigid3d& global_pose);

  MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()
      const;
  PoseGraphInterface::SubmapData GetSubmapDataUnderLock(
      const SubmapId& submap_id) const;
};

void PoseGraphMap::LoadSubmapList(const Json::Value& map_data) {
  overlap_computer_->LoadSubmapList(map_data);
}

Json::Value PoseGraphMap::GetSubmapListToJson() {
  return overlap_computer_->GetSubmapListToJson();
}

void PoseGraphMap::LoadStateFromProto(
    io::ProtoStreamReaderInterface* const reader) {
  io::ProtoStreamDeserializer deserializer(reader);
  header_ = deserializer.header();
  *pose_graph_proto_.mutable_pose_graph() = deserializer.pose_graph();
  *trajectory_builder_options_proto_.mutable_all_trajectory_builder_options() =
      deserializer.all_trajectory_builder_options();

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();
  // remapping trajectory_id
  std::map<int, int> trajectory_remapping;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    trajectory_id_.push_back(new_trajectory_id);
  }
  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    SetLandmarkPose(landmark.landmark_id(),
                    transform::ToRigid3(landmark.global_pose()));
  }

  int submap_num = 0;

  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        AddSubmapFromProto(submap_poses.at(submap_id), proto.submap());
        submaps_proto_.push_back(proto);
        submap_num++;
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        AddNodeFromProto(node_pose, proto.node());
        trajectory_node_proto_.push_back(proto);
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        SetTrajectoryDataFromProto(proto.trajectory_data());
        all_trajectory_data_proto_.push_back(proto);
        break;
      }
      case SerializedData::kImuData: {
        break;
      }
      case SerializedData::kOdometryData: {
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        break;
      }
      case SerializedData::kLandmarkData: {
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  LOG(WARNING) << "Load pbstream with submaps size: " << submap_num;

  AddSerializedConstraints(FromProto(pose_graph_proto.constraint()));
}

int PoseGraphMap::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  absl::MutexLock locker(&mutex_);
  const int trajectory_id = all_trajectory_builder_options_.size();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  return trajectory_id;
}

void PoseGraphMap::AddSubmapFromProto(
    const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {
  if (!submap.has_submap_2d()) return;

  const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                              submap.submap_id().submap_index()};
  const transform::Rigid2d global_submap_pose_2d =
      transform::Project2D(global_submap_pose);
  {
    absl::MutexLock locker(&mutex_);
    const std::shared_ptr<const Submap2D> submap_ptr =
        std::make_shared<const Submap2D>(submap.submap_2d(),
                                         &conversion_tables_);
    submap_data_.Insert(submap_id, InternalSubmapData());
    submap_data_.at(submap_id).submap = submap_ptr;
    // Immediately show the submap at the 'global_submap_pose'.
    global_submap_poses_2d_.Insert(submap_id, global_submap_pose_2d);
    LOG(INFO) << "AddSubmapFromProto id:" << submap_id
              << "; global_submap_pose_2d:" << global_submap_pose_2d;
  }
}

void PoseGraphMap::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                    const proto::Node& node) {
  const NodeId node_id = {node.node_id().trajectory_id(),
                          node.node_id().node_index()};
  std::shared_ptr<const TrajectoryNode::Data> constant_data =
      std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));
  {
    absl::MutexLock locker(&mutex_);
    trajectory_nodes_.Insert(node_id,
                             TrajectoryNode{constant_data, global_pose});
  }
}

void PoseGraphMap::SetTrajectoryDataFromProto(
    const proto::TrajectoryData& data) {
  absl::MutexLock locker(&mutex_);
  TrajectoryData trajectory_data;
  // gravity_constant and imu_calibration are omitted as its not used in 2d
  if (data.has_fixed_frame_origin_in_map()) {
    trajectory_data.fixed_frame_origin_in_map =
        transform::ToRigid3(data.fixed_frame_origin_in_map());
    const int trajectory_id = data.trajectory_id();
    trajectory_data_[trajectory_id] = trajectory_data;
  }
}

void PoseGraphMap::SetLandmarkPose(const std::string& landmark_id,
                                   const transform::Rigid3d& global_pose) {
  landmark_nodes_[landmark_id].global_landmark_pose = global_pose;
}

void PoseGraphMap::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  absl::MutexLock locker(&mutex_);
  for (const auto& constraint : constraints) {
    if (!trajectory_nodes_.Contains(constraint.node_id)) {
      LOG(INFO) << "AddSerializedConstraints faild in node_id "
                << constraint.node_id << " in submap_id "
                << constraint.submap_id << " constraint tag " << constraint.tag;
    }
    CHECK(trajectory_nodes_.Contains(constraint.node_id));
    CHECK(submap_data_.Contains(constraint.submap_id));
    CHECK(trajectory_nodes_.at(constraint.node_id).constant_data != nullptr);
    CHECK(submap_data_.at(constraint.submap_id).submap != nullptr);
    switch (constraint.tag) {
      case Constraint::Tag::INTRA_SUBMAP:
        CHECK(submap_data_.at(constraint.submap_id)
                  .node_ids.emplace(constraint.node_id)
                  .second);
        break;
      case Constraint::Tag::INTER_SUBMAP:
        // UpdateTrajectoryConnectivity(constraint);
        break;
    }
    const Constraint::Pose pose = {
        constraint.pose.zbar_ij *
            transform::Rigid3d::Rotation(
                trajectory_nodes_.at(constraint.node_id)
                    .constant_data->gravity_alignment.inverse()),
        constraint.pose.translation_weight, constraint.pose.rotation_weight};
    constraints_.push_back(Constraint{constraint.submap_id, constraint.node_id,
                                      pose, constraint.tag});
  }
  LOG(INFO) << "Loaded " << constraints_.size() << " constraints_.";
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraphMap::GetAllSubmapData() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraphMap::GetSubmapDataUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : submap_data_) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

PoseGraphInterface::SubmapData PoseGraphMap::GetSubmapData(
    const SubmapId& submap_id) const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

PoseGraphInterface::SubmapData PoseGraphMap::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = submap_data_.find(submap_id);
  if (it == submap_data_.end()) {
    return {};
  }
  auto submap = it->data.submap;
  if (global_submap_poses_2d_.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap, transform::Embed3D(global_submap_poses_2d_.at(submap_id))};
  } else {
    return {};
  }
}

MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraphMap::GetAllSubmapPoses() const {
  absl::MutexLock locker(&mutex_);
  MapById<SubmapId, PoseGraphInterface::SubmapPose> submap_poses;
  for (const auto& submap_id_data : submap_data_) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        PoseGraphInterface::SubmapPose{submap_data.submap->num_range_data(),
                                       submap_data.pose});
  }
  return submap_poses;
}

MapById<NodeId, TrajectoryNode> PoseGraphMap::GetTrajectoryNodes() const {
  absl::MutexLock locker(&mutex_);
  return trajectory_nodes_;
}

MapById<NodeId, TrajectoryNodePose> PoseGraphMap::GetTrajectoryNodePoses()
    const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& node_id_data : trajectory_nodes_) {
    absl::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<std::string, PoseGraphInterface::LandmarkNode> PoseGraphMap::GetLandmarkNodes() const {
  absl::MutexLock locker(&mutex_);
  return landmark_nodes_;
}

std::map<std::string, transform::Rigid3d> PoseGraphMap::GetLandmarkPoses() const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& landmark : landmark_nodes_) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] = landmark.second.global_landmark_pose.value();
  }
  return landmark_poses;
}

std::vector<Constraint> PoseGraphMap::constraints() const {
  std::vector<Constraint> result;
  absl::MutexLock locker(&mutex_);
  for (const Constraint& constraint : constraints_) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 trajectory_nodes_.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

int PoseGraphMap::num_trajectory() const {
  absl::MutexLock locker(&mutex_);
  return all_trajectory_builder_options_.size();
}

void PoseGraphMap::OptimizeSubmapPose(const SubmapId& submap_id,
                                      const std::array<double, 3>& input_pose) {
  std::vector<std::array<double, 3>> submap_pose;
  std::vector<transform::Rigid3d> submap_tf;

  {
    absl::MutexLock locker(&mutex_);
    for (const auto& submap_id_data : submap_data_) {
      auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
      const auto& local_pose = submap_data.pose;
      const transform::Rigid3d cur_tf = {
          {local_pose.translation().x(), local_pose.translation().y(),
           local_pose.translation().z()},
          Eigen::Quaterniond(
              local_pose.rotation().w(), local_pose.rotation().x(),
              local_pose.rotation().y(), local_pose.rotation().z())};

      if (submap_id_data.id == submap_id) {
        submap_pose.push_back({input_pose[0], input_pose[1], input_pose[2]});
      } else {
        submap_pose.push_back({cur_tf.translation().x(),
                               cur_tf.translation().y(),
                               transform::GetYaw(cur_tf.rotation())});
      }
      submap_tf.push_back(cur_tf);
    }
  }

  carto_map::OptimizePose(submap_pose, submap_tf);

  {
    absl::MutexLock locker(&mutex_);
    int submap_cnt = 0;
    for (const auto& submap_id_data : submap_data_) {
      global_submap_poses_2d_.at(submap_id_data.id) = transform::Rigid2d(
          {submap_pose[submap_cnt][0], submap_pose[submap_cnt][1]},
          submap_pose[submap_cnt][2]);
      LOG(INFO) << "OptimizeSubmapPose id:" << submap_id_data.id
                << "; global_submap_pose_2d:"
                << global_submap_poses_2d_.at(submap_id_data.id);
      submap_cnt++;
    }
  }
}

void PoseGraphMap::ChangeSubmapPose(const SubmapId& submap_id,
                                    const std::array<double, 3>& input_pose) {
  absl::MutexLock locker(&mutex_);
  global_submap_poses_2d_.at(submap_id) =
      transform::Rigid2d({input_pose[0], input_pose[1]}, input_pose[2]);
}

std::array<double, 3> PoseGraphMap::GetSubmapPose(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  auto res_pose = transform::Embed3D(global_submap_poses_2d_.at(submap_id));
  std::array<double, 3> res = {res_pose.translation().x(),
                               res_pose.translation().y(),
                               transform::GetYaw(res_pose.rotation())};
  return res;
}

proto::PoseGraph PoseGraphMap::ToProto() const {
  proto::PoseGraph proto;

  std::map<int, proto::Trajectory* const> trajectory_protos;
  const auto trajectory = [&proto, &trajectory_protos](
                              const int trajectory_id) -> proto::Trajectory* {
    if (trajectory_protos.count(trajectory_id) == 0) {
      auto* const trajectory_proto = proto.add_trajectory();
      trajectory_proto->set_trajectory_id(trajectory_id);
      LOG(INFO) << "Add trajectory_id to PoseGraph Proto id:" << trajectory_id;
      CHECK(trajectory_protos.emplace(trajectory_id, trajectory_proto).second);
    }
    return trajectory_protos.at(trajectory_id);
  };

  for (const auto& submap_id_data : GetAllSubmapData()) {
    // LOG(INFO) << "Add submap_id to PoseGraph Proto id:" << submap_id_data.id;
    proto::Trajectory* trajectory_proto =
        trajectory(submap_id_data.id.trajectory_id);
    CHECK(submap_id_data.data.submap != nullptr);
    auto* const submap_proto = trajectory_proto->add_submap();
    submap_proto->set_submap_index(submap_id_data.id.submap_index);
    *submap_proto->mutable_pose() =
        transform::ToProto(submap_id_data.data.pose);
  }

  auto constraints_copy = constraints();
  std::set<mapping::NodeId> orphaned_nodes;
  proto.mutable_constraint()->Reserve(constraints_copy.size());
  for (auto it = constraints_copy.begin(); it != constraints_copy.end();) {
    *proto.add_constraint() = cartographer::mapping::ToProto(*it);
    ++it;
  }

  for (const auto& node_id_data : GetTrajectoryNodes()) {
    // LOG(INFO) << "Add node_id to PoseGraph Proto id:" << node_id_data.id;
    proto::Trajectory* trajectory_proto =
        trajectory(node_id_data.id.trajectory_id);
    CHECK(node_id_data.data.constant_data != nullptr);
    auto* const node_proto = trajectory_proto->add_node();
    node_proto->set_node_index(node_id_data.id.node_index);
    node_proto->set_timestamp(
        common::ToUniversal(node_id_data.data.constant_data->time));
    *node_proto->mutable_pose() =
        transform::ToProto(node_id_data.data.global_pose);
  }

  return proto;
}

mapping::proto::AllTrajectoryBuilderOptions
PoseGraphMap::GetAllTrajectoryBuilderOptions() const {
  absl::MutexLock locker(&mutex_);
  mapping::proto::AllTrajectoryBuilderOptions all_options_proto;
  for (auto& id : trajectory_id_) {
    if (trajectory_to_delete_.count(id) != 0) continue;
    *all_options_proto.add_options_with_sensor_ids() =
        all_trajectory_builder_options_[id];
  }
  return all_options_proto;
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraphMap::GetTrajectoryData() const {
  absl::MutexLock locker(&mutex_);
  return trajectory_data_;
}

void PoseGraphMap::RemoveSubmap(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (!submap_data_.Contains(submap_id)) {
    LOG(WARNING) << "Remove Submap donot exit" << submap_id;
    return;
  }
  LOG(INFO) << "RemoveSubmap " << submap_id;
  // LOG(INFO) << "Trajectory " << submap_id.trajectory_id << " have "
  //           << submap_data_.SizeOfTrajectoryOrZero(submap_id.trajectory_id)
  //           << " submaps";

  std::set<NodeId> nodes_to_retain;
  for (const auto& submap_data : submap_data_) {
    if (submap_data.id != submap_id) {
      nodes_to_retain.insert(submap_data.data.node_ids.begin(),
                             submap_data.data.node_ids.end());
    }
  }
  // Remove all nodes that are exlusively associated to 'submap_id'.
  std::set<NodeId> nodes_to_remove;
  std::set_difference(submap_data_.at(submap_id).node_ids.begin(),
                      submap_data_.at(submap_id).node_ids.end(),
                      nodes_to_retain.begin(), nodes_to_retain.end(),
                      std::inserter(nodes_to_remove, nodes_to_remove.begin()));
  // Remove all 'data_.constraints' related to 'submap_id'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : constraints_) {
      if (constraint.submap_id != submap_id) {
        constraints.push_back(constraint);
      }
    }
    constraints_ = std::move(constraints);
  }

  // Remove all 'data_.constraints' related to 'nodes_to_remove'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : constraints_) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    constraints_ = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  submap_data_.Trim(submap_id);

  // Remove the 'nodes_to_remove' from the pose graph
  for (const NodeId& node_id : nodes_to_remove) {
    trajectory_nodes_.Trim(node_id);
    // LOG(INFO) << "Remove node_id " << node_id << " in trajectory_nodes_";
  }

  LOG(INFO) << "Trajectory " << submap_id.trajectory_id << " have "
            << submap_data_.SizeOfTrajectoryOrZero(submap_id.trajectory_id)
            << " submaps left#####";
  if (submap_data_.SizeOfTrajectoryOrZero(submap_id.trajectory_id) == 0) {
    trajectory_to_delete_.insert(submap_id.trajectory_id);
  }
  if (trajectory_nodes_.SizeOfTrajectoryOrZero(submap_id.trajectory_id) == 0) {
    trajectory_data_.erase(submap_id.trajectory_id);
  }
}

void PoseGraphMap::RemoveTrajectory(int trajectory_id) {
  LOG(INFO) << "RemoveTrajectory " << trajectory_id;
  std::vector<SubmapId> submap_ids;
  for (const auto& submap : submap_data_.trajectory(trajectory_id)) {
    submap_ids.push_back(submap.id);
  }
  for (auto& submap_id : submap_ids) {
    RemoveSubmap(submap_id);
  }
}

void PoseGraphMap::DrawSubmap(const SubmapId& submap_id,
                              const std::string& file_name) {
  if (!submap_data_.Contains(submap_id)) {
    LOG(WARNING) << "Submap donot exit" << submap_id;
    return;
  }
  LOG(INFO) << "DrawSubmap " << submap_id << " to file : " << file_name;
  const Grid2D& tar_grid = *std::static_pointer_cast<const Submap2D>(
                                submap_data_.at(submap_id).submap)
                                ->grid();
  submap_matcher_->SaveImageFromGrid(tar_grid, file_name);
}

void PoseGraphMap::WriteProto(io::ProtoStreamWriterInterface* const writer) {
  writer->WriteProto(header_);
  writer->WriteProto(pose_graph_proto_);
  writer->WriteProto(trajectory_builder_options_proto_);
  for (auto& proto : submaps_proto_) {
    writer->WriteProto(proto);
  }
  for (auto& proto : trajectory_node_proto_) {
    writer->WriteProto(proto);
  }
  for (auto& proto : all_trajectory_data_proto_) {
    writer->WriteProto(proto);
  }
}

std::vector<SubmapId> PoseGraphMap::ComputeOverlappedSubmaps() {
  // WriteNodePoseStampToFile("/home/dean/Pictures/node_poses.txt");
  auto submap_data = GetAllSubmapData();
  return overlap_computer_->ComputeAllSubmapsOverlap(submap_data);
}

void PoseGraphMap::WriteNodePoseStampToFile(const std::string& file_name) {
  std::ofstream out_file(file_name, std::ios::out | std::ios::binary);
  for (const auto& node_id_data : GetTrajectoryNodes()) {
    out_file << std::fixed << std::setprecision(8)
             << node_id_data.data.time().time_since_epoch().count() / 10000000.0
             << " " << node_id_data.data.global_pose.translation().x() << " "
             << node_id_data.data.global_pose.translation().y() << " "
             << node_id_data.data.global_pose.translation().z() << " "
             << node_id_data.data.global_pose.rotation().x() << " "
             << node_id_data.data.global_pose.rotation().y() << " "
             << node_id_data.data.global_pose.rotation().z() << " "
             << node_id_data.data.global_pose.rotation().w() << std::endl;
    // out_file << "Trajectory_id:(" << node_id_data.id.trajectory_id << "," <<
    // node_id_data.id.node_index
    //           << "),Time:" << node_id_data.data.time()
    //           << ",Pose:[" << node_id_data.data.global_pose.translation().x()
    //           << ","
    //                        << node_id_data.data.global_pose.translation().y()
    //                        << ","
    //                        << node_id_data.data.global_pose.translation().z()
    //                        << ","
    //                        << node_id_data.data.global_pose.rotation().w() <<
    //                        ","
    //                        << node_id_data.data.global_pose.rotation().x() <<
    //                        ","
    //                        << node_id_data.data.global_pose.rotation().y() <<
    //                        ","
    //                        << node_id_data.data.global_pose.rotation().z() <<
    //                        "]" << std::endl;
  }
  out_file.close();
}

void PoseGraphMap::SubmapPoseOptimization(const SubmapId& input_submap_id,
                                          const SubmapId& source_submap_id) {
  transform::Rigid3d global_frame_from_local_frame1 =
      transform::Embed3D(global_submap_poses_2d_.at(input_submap_id)) *
      submap_data_.at(input_submap_id).submap->local_pose().inverse();
  transform::Rigid3d global_frame_from_local_frame2 =
      transform::Embed3D(global_submap_poses_2d_.at(source_submap_id)) *
      submap_data_.at(source_submap_id).submap->local_pose().inverse();

  transform::Rigid2d pose_prediction =
      transform::Project2D(global_frame_from_local_frame2.inverse() *
                           global_frame_from_local_frame1);
  transform::Rigid2d pose_estimate = pose_prediction;

  const Grid2D& input_grid = *std::static_pointer_cast<const Submap2D>(
                                  submap_data_.at(input_submap_id).submap)
                                  ->grid();
  const Grid2D& source_grid = *std::static_pointer_cast<const Submap2D>(
                                   submap_data_.at(source_submap_id).submap)
                                   ->grid();

  pcl_matcher_->MatchICP(input_grid, source_grid, pose_prediction,
                         &pose_estimate);

  // submap_matcher_->MatchCSM(input_grid, source_grid, pose_prediction,
  //                           &pose_estimate);

  LOG(INFO) << "global_frame_from_local_frame1"
            << global_frame_from_local_frame1;
  LOG(INFO) << "global_frame_from_local_frame2"
            << global_frame_from_local_frame2;
  LOG(INFO) << "pose_prediction" << pose_prediction;
  LOG(INFO) << "pose_estimate" << pose_estimate;

  {
    absl::MutexLock locker(&mutex_);
    global_submap_poses_2d_.at(input_submap_id) = transform::Project2D(
        global_frame_from_local_frame2 * transform::Embed3D(pose_estimate) *
        submap_data_.at(input_submap_id).submap->local_pose());
    LOG(INFO) << "OptimizeSubmapPose id:" << input_submap_id
              << "; global_submap_pose_2d:"
              << global_submap_poses_2d_.at(input_submap_id);
  }

  // submap_matcher_->SaveImageFromMatchGrid(input_grid, source_grid,
  //                                         global_frame_from_local_frame1,
  //                                         global_frame_from_local_frame2,
  //                                         "/home/dean/Pictures/submap.png");

  // pcl_matcher_->SaveImageFromMatchGrid(input_grid, source_grid,
  //                                       global_frame_from_local_frame1,
  //                                       global_frame_from_local_frame2,
  //                                       "/home/dean/Pictures/submap.png");
}

}  // namespace mapping
}  // namespace cartographer
