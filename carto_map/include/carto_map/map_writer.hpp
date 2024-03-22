#include "carto_map/pose_graph.hpp"
#include "cartographer/mapping/proto/serialization.pb.h"

namespace cartographer {
namespace io {

using mapping::proto::SerializedData;

void WriteMapProto(const cartographer::mapping::PoseGraphMap& pose_graph,
                   ProtoStreamWriterInterface* const writer) {
  mapping::proto::SerializationHeader header;
  header.set_format_version(2);
  writer->WriteProto(header);
  // LOG(INFO) << "WriteMapProto header success";

  SerializedData pose_graph_proto;
  *pose_graph_proto.mutable_pose_graph() = pose_graph.ToProto();
  writer->WriteProto(pose_graph_proto);
  // LOG(INFO) << "WriteMapProto pose_graph_proto success";

  SerializedData trajectory_builder_options_proto;
  *trajectory_builder_options_proto.mutable_all_trajectory_builder_options() =
      pose_graph.GetAllTrajectoryBuilderOptions();
  writer->WriteProto(trajectory_builder_options_proto);
  // LOG(INFO) << "WriteMapProto trajectory_builder_options_proto success";

  for (const auto& submap_id_data : pose_graph.GetAllSubmapData()) {
    SerializedData submaps_proto;
    auto* const submap_proto = submaps_proto.mutable_submap();
    *submap_proto = submap_id_data.data.submap->ToProto(true);
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    writer->WriteProto(submaps_proto);
  }
  // LOG(INFO) << "WriteMapProto submaps_proto success";

  for (const auto& node_id_data : pose_graph.GetTrajectoryNodes()) {
    SerializedData trajectory_node_proto;
    auto* const node_proto = trajectory_node_proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(trajectory_node_proto);
  }
  // LOG(INFO) << "WriteMapProto trajectory_node_proto success";

  for (const auto& trajectory_data : pose_graph.GetTrajectoryData()) {
    SerializedData all_trajectory_data_proto;
    auto* const trajectory_data_proto =
        all_trajectory_data_proto.mutable_trajectory_data();
    trajectory_data_proto->set_trajectory_id(trajectory_data.first);
    trajectory_data_proto->set_gravity_constant(
        trajectory_data.second.gravity_constant);
    *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
        Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                           trajectory_data.second.imu_calibration[1],
                           trajectory_data.second.imu_calibration[2],
                           trajectory_data.second.imu_calibration[3]));
    if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
      *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
          transform::ToProto(
              trajectory_data.second.fixed_frame_origin_in_map.value());
    }
    writer->WriteProto(all_trajectory_data_proto);
  }

  for (const auto& node : pose_graph.GetLandmarkNodes()) {
    for (const auto& observation : node.second.landmark_observations) {
      SerializedData landmark_observations_proto;
      auto* landmark_data_proto = landmark_observations_proto.mutable_landmark_data();
      landmark_data_proto->set_trajectory_id(observation.trajectory_id);
      landmark_data_proto->mutable_landmark_data()->set_timestamp(
          common::ToUniversal(observation.time));
      auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                    ->add_landmark_observations();
      observation_proto->set_id(node.first);
      *observation_proto->mutable_landmark_to_tracking_transform() =
          transform::ToProto(observation.landmark_to_tracking_transform);
      observation_proto->set_translation_weight(observation.translation_weight);
      observation_proto->set_rotation_weight(observation.rotation_weight);
      writer->WriteProto(landmark_observations_proto);
    }
  }

  LOG(INFO) << "WriteMapProto all_trajectory_data_proto success";
}

}  // namespace io
}  // namespace cartographer
