#include "absl/synchronization/mutex.h"
#include "carto_map/map_manager.hpp"
#include "glog/logging.h"
#include "interactive_markers/interactive_marker_server.h"
#include "interactive_markers/menu_handler.h"
#include "ros/ros.h"
#include "tf2/utils.h"

class NodeMain {
 public:
  NodeMain();
  ~NodeMain();

  void Init();

  ::ros::NodeHandle* node_handle() { return &node_handle_; }

  void processInteractiveFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

 private:
  /* data */
  absl::Mutex mutex_;
  std::string map_file;

  absl::Mutex move_node_mutex_;
  bool interactive_mode_;
  std::map<std::string, std::array<double, 3>> moved_nodes_;

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;

  ::ros::ServiceServer submap_query_server_;
  ::ros::ServiceServer save_map_server_;
  ::ros::ServiceServer optimize_submap_server_;
  ::ros::ServiceServer optimize_submap_pose_server_;
  ::ros::ServiceServer get_submap_pose_server_;
  ::ros::ServiceServer remove_submap_server_;
  ::ros::ServiceServer remove_trajectory_server_;
  ::ros::ServiceServer compute_overlap_submap_server_;
  ::ros::ServiceServer interactive_mode_server_;
  ::ros::ServiceServer clear_move_nodes_server_;

  ::ros::WallTimer submap_list_pub_timer_;
  ::ros::WallTimer trajectory_node_list_pub_timer_;
  ::ros::WallTimer constraint_list_pub_timer_;
  ::ros::WallTimer landmark_poses_list_pub_timer_;

  std::unique_ptr<MapManager> map_manager_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_server_;

  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);

  bool HandleSubmapQuery(
      ::cartographer_ros_msgs::SubmapQuery::Request& request,
      ::cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleSaveMap(carto_map::SaveMap::Request& request,
                     carto_map::SaveMap::Response& response);
  bool HandleOptimizeSubmap(carto_map::OptimizeSubmap::Request& request,
                            carto_map::OptimizeSubmap::Response& response);
  bool HandleOptimizeSubmapPose(
      carto_map::OptimizeSubmapPose::Request& request,
      carto_map::OptimizeSubmapPose::Response& response);
  bool HandleGetSubmapPose(carto_map::GetSubmapPose::Request& request,
                           carto_map::GetSubmapPose::Response& response);
  bool HandleRemoveSubmap(carto_map::RemoveSubmap::Request& request,
                          carto_map::RemoveSubmap::Response& response);
  bool HandleRemoveTrajectory(carto_map::RemoveTrajectory::Request& request,
                              carto_map::RemoveTrajectory::Response& response);
  bool HandleComputeOverlapSubmap(std_srvs::Trigger::Request& request,
                                  std_srvs::Trigger::Response& response);
  bool SetInteractive(carto_map::SetInteractiveMode::Request& request,
                      carto_map::SetInteractiveMode::Response& response);
  bool HandleClearMoveNodes(std_srvs::Trigger::Request& request,
                            std_srvs::Trigger::Response& response);

  void PublishInteractiveMark();
  void clearMovedNodes(bool recover_change);
  void addMovedNodes(const std::string& id, std::array<double, 3> vec);

  visualization_msgs::InteractiveMarker toInteractiveMarker(
      const std::string& id, visualization_msgs::Marker& marker,
      const double& scale);
  visualization_msgs::Marker toVertexMarker(const std::string& frame,
                                            const std::string& ns,
                                            const double& scale);
};

NodeMain::NodeMain() {}

NodeMain::~NodeMain() {}

void NodeMain::Init() {
  interactive_mode_ = false;
  map_manager_ = absl::make_unique<MapManager>("map");
  bool show_disable_submap;
  node_handle_.param("pbstream_file_path", map_file, std::string("/home/map"));
  node_handle_.param("show_disable_submap", show_disable_submap, true);
  if (!map_manager_->LoadMap(map_file, show_disable_submap)) {
    ROS_ERROR("Failed to load the map in %s", map_file.c_str());
    return;
  }

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          "/submap_list", 1);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          "/trajectory_node_list", 1);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          "/constraint_list", 1);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          "/landmark_poses_list", 1);

  submap_query_server_ = node_handle_.advertiseService(
      "submap_query", &NodeMain::HandleSubmapQuery, this);

  save_map_server_ = node_handle_.advertiseService(
      "/carto_map/save_map", &NodeMain::HandleSaveMap, this);
  optimize_submap_server_ = node_handle_.advertiseService(
      "/carto_map/optimize_submap", &NodeMain::HandleOptimizeSubmap, this);
  optimize_submap_pose_server_ =
      node_handle_.advertiseService("/carto_map/optimize_pose_submap",
                                    &NodeMain::HandleOptimizeSubmapPose, this);
  get_submap_pose_server_ = node_handle_.advertiseService(
      "/carto_map/get_submap_pose", &NodeMain::HandleGetSubmapPose, this);
  remove_submap_server_ = node_handle_.advertiseService(
      "/carto_map/remove_submap", &NodeMain::HandleRemoveSubmap, this);
  remove_trajectory_server_ = node_handle_.advertiseService(
      "/carto_map/remove_trajectory", &NodeMain::HandleRemoveTrajectory, this);
  compute_overlap_submap_server_ = node_handle_.advertiseService(
      "/carto_map/compute_overlap_submap",
      &NodeMain::HandleComputeOverlapSubmap, this);
  interactive_mode_server_ = node_handle_.advertiseService(
      "/carto_map/set_interactive_mode", &NodeMain::SetInteractive, this);
  clear_move_nodes_server_ = node_handle_.advertiseService(
      "/carto_map/clear_move_nodes", &NodeMain::HandleClearMoveNodes, this);

  interactive_server_ =
      std::make_unique<interactive_markers::InteractiveMarkerServer>(
          "carto_map", "", true);

  submap_list_pub_timer_ = node_handle_.createWallTimer(
      ::ros::WallDuration(0.3), &NodeMain::PublishSubmapList, this);
  trajectory_node_list_pub_timer_ = node_handle_.createWallTimer(
      ::ros::WallDuration(30e-3), &NodeMain::PublishTrajectoryNodeList, this);
  landmark_poses_list_pub_timer_ = node_handle_.createWallTimer(
      ::ros::WallDuration(30e-3), &NodeMain::PublishLandmarkPosesList, this);
  constraint_list_pub_timer_ = node_handle_.createWallTimer(
      ::ros::WallDuration(0.5), &NodeMain::PublishConstraintList, this);
}

void NodeMain::PublishSubmapList(const ::ros::WallTimerEvent& timer_event) {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_.publish(map_manager_->GetSubmapList());
}

void NodeMain::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_manager_->GetTrajectoryNodeList());
  }
}

void NodeMain::PublishConstraintList(const ::ros::WallTimerEvent& timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_.publish(map_manager_->GetConstraintList());
  }
}

void NodeMain::PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_.publish(map_manager_->GetLandmarkPosesList());
  }
}

bool NodeMain::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->HandleSubmapQuery(request, response);
  return true;
}

bool NodeMain::HandleSaveMap(carto_map::SaveMap::Request& request,
                             carto_map::SaveMap::Response& response) {
  absl::MutexLock lock(&mutex_);
  if (request.filename == "###") {
    map_manager_->SaveMap(map_file);
    map_manager_->SaveMapInfo(map_file);
  } else {
    map_manager_->SaveMap(request.filename);
    map_manager_->SaveMapInfo(request.filename);
  }
  return true;
}

bool NodeMain::HandleOptimizeSubmap(
    carto_map::OptimizeSubmap::Request& request,
    carto_map::OptimizeSubmap::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->HandleOptimizeSubmap(request, response);
  return true;
}

bool NodeMain::HandleOptimizeSubmapPose(
    carto_map::OptimizeSubmapPose::Request& request,
    carto_map::OptimizeSubmapPose::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->HandleOptimizeSubmapPose(request, response);
  return true;
}

bool NodeMain::HandleRemoveSubmap(carto_map::RemoveSubmap::Request& request,
                                  carto_map::RemoveSubmap::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->HandleRemoveSubmap(request, response);
  return true;
}

bool NodeMain::HandleRemoveTrajectory(
    carto_map::RemoveTrajectory::Request& request,
    carto_map::RemoveTrajectory::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->HandleRemoveTrajectory(request, response);
  return true;
}

bool NodeMain::HandleComputeOverlapSubmap(
    std_srvs::Trigger::Request& request,
    std_srvs::Trigger::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->ComputeOverlappedSubmaps();
  return true;
}

bool NodeMain::HandleGetSubmapPose(
    carto_map::GetSubmapPose::Request& request,
    carto_map::GetSubmapPose::Response& response) {
  absl::MutexLock lock(&mutex_);
  std::array<double, 3> res = map_manager_->GetSubmapPoseByID(
      request.trajectory_id, request.submap_index);
  response.x = res[0];
  response.y = res[1];
  response.theta = res[2];
  return true;
}

bool NodeMain::HandleClearMoveNodes(std_srvs::Trigger::Request& request,
                                    std_srvs::Trigger::Response& response) {
  absl::MutexLock lock(&mutex_);
  clearMovedNodes(true);
  PublishInteractiveMark();
}

bool NodeMain::SetInteractive(
    carto_map::SetInteractiveMode::Request& request,
    carto_map::SetInteractiveMode::Response& response) {
  absl::MutexLock lock(&mutex_);
  LOG(INFO) << "SetInteractiveMode:" << request.cmd;
  bool cmd = request.cmd;
  if (interactive_mode_ == cmd) return true;

  interactive_mode_ = cmd;
  PublishInteractiveMark();
  return true;
}

void NodeMain::PublishInteractiveMark() {
  interactive_server_->clear();
  if (interactive_mode_) {
    auto submap_poses = map_manager_->GetAllSubmapPoses();
    for (const auto& submap_pose : submap_poses) {
      visualization_msgs::Marker vertex_marker =
          toVertexMarker("map", "carto_map", 0.1);
      vertex_marker.pose = submap_pose.second;
      visualization_msgs::InteractiveMarker int_marker =
          toInteractiveMarker(submap_pose.first, vertex_marker, 0.3);
      interactive_server_->insert(
          int_marker,
          boost::bind(&NodeMain::processInteractiveFeedback, this, _1));
    }
  }
  interactive_server_->applyChanges();
}

void NodeMain::processInteractiveFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  std::string id = feedback->marker_name;
  // was depressed, something moved, and now released
  if (feedback->event_type ==
          visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP &&
      feedback->mouse_point_valid) {
    addMovedNodes(id, {feedback->mouse_point.x, feedback->mouse_point.y,
                       tf2::getYaw(feedback->pose.orientation)});
  }
}

void NodeMain::clearMovedNodes(bool recover_change) {
  absl::MutexLock lock(&move_node_mutex_);
  if (recover_change) {
    for (const auto& node_pose : moved_nodes_) {
      int trajectory_id;
      int submap_index;
      char comma;
      // 从字符串流中提取trajectory_id和submap_index
      std::istringstream ss(node_pose.first);
      ss >> trajectory_id >> comma >> submap_index;
      if (ss.fail() || comma != ',') {
        LOG(ERROR) << "Error format of submap_id!";
      } else {
        map_manager_->ChangeSubmapPoseByID(trajectory_id, submap_index,
                                           node_pose.second);
      }
    }
  }
  moved_nodes_.clear();
}

void NodeMain::addMovedNodes(const std::string& id, std::array<double, 3> vec) {
  absl::MutexLock lock(&move_node_mutex_);
  int trajectory_id;
  int submap_index;
  char comma;
  // 从字符串流中提取trajectory_id和submap_index
  std::istringstream ss(id);
  ss >> trajectory_id >> comma >> submap_index;

  if (ss.fail() || comma != ',') {
    LOG(ERROR) << "Error format of submap_id!";
  } else {
    if (moved_nodes_.count(id) == 0) {
      std::array<double, 3> res =
          map_manager_->GetSubmapPoseByID(trajectory_id, submap_index);
      moved_nodes_[id] = res;
    }
    map_manager_->ChangeSubmapPoseByID(trajectory_id, submap_index, vec);
  }
}

visualization_msgs::InteractiveMarker NodeMain::toInteractiveMarker(
    const std::string& id, visualization_msgs::Marker& marker,
    const double& scale) {
  // marker basics
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = id;
  int_marker.pose.orientation.w = 1.;
  int_marker.pose.position.x = marker.pose.position.x;
  int_marker.pose.position.y = marker.pose.position.y;
  int_marker.scale = scale;

  // translate control
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode =
      visualization_msgs::InteractiveMarkerControl::FIXED;
  control.always_visible = true;
  control.orientation.w = 0;
  control.orientation.x = 0.7071;
  control.orientation.y = 0;
  control.orientation.z = 0.7071;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  // rotate control
  visualization_msgs::InteractiveMarkerControl control_rot;
  control_rot.orientation_mode =
      visualization_msgs::InteractiveMarkerControl::FIXED;
  control_rot.always_visible = true;
  control_rot.orientation.w = 0;
  control_rot.orientation.x = 0.7071;
  control_rot.orientation.y = 0;
  control_rot.orientation.z = 0.7071;
  control_rot.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control_rot);

  return int_marker;
}

visualization_msgs::Marker NodeMain::toVertexMarker(const std::string& frame,
                                                    const std::string& ns,
                                                    const double& scale) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 0;
  marker.color.b = 0.0;
  marker.color.a = 1.;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.);

  return marker;
}
