#include "absl/synchronization/mutex.h"
#include "carto_map/map_manager.hpp"
#include "glog/logging.h"
#include "ros/ros.h"

class NodeMain {
 public:
  NodeMain();
  ~NodeMain();

  void Init();

  ::ros::NodeHandle* node_handle() { return &node_handle_; }

 private:
  /* data */
  absl::Mutex mutex_;

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;

  ::ros::ServiceServer submap_query_server_;
  ::ros::ServiceServer save_map_server_;
  ::ros::ServiceServer optimize_submap_server_;
  ::ros::ServiceServer remove_submap_server_;
  ::ros::ServiceServer remove_trajectory_server_;
  ::ros::ServiceServer compute_overlap_submap_server_;

  ::ros::WallTimer submap_list_pub_timer_;
  ::ros::WallTimer trajectory_node_list_pub_timer_;
  ::ros::WallTimer constraint_list_pub_timer_;

  std::unique_ptr<MapManager> map_manager_;

  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);

  bool HandleSubmapQuery(
      ::cartographer_ros_msgs::SubmapQuery::Request& request,
      ::cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleSaveMap(carto_map::SaveMap::Request& request,
                     carto_map::SaveMap::Response& response);
  bool HandleOptimizeSubmap(carto_map::OptimizeSubmap::Request& request,
                            carto_map::OptimizeSubmap::Response& response);
  bool HandleRemoveSubmap(carto_map::RemoveSubmap::Request& request,
                          carto_map::RemoveSubmap::Response& response);
  bool HandleRemoveTrajectory(carto_map::RemoveTrajectory::Request& request,
                              carto_map::RemoveTrajectory::Response& response);
  bool HandleComputeOverlapSubmap(std_srvs::Trigger::Request& request,
                                  std_srvs::Trigger::Response& response);
};

NodeMain::NodeMain() {}

NodeMain::~NodeMain() {}

void NodeMain::Init() {
  map_manager_ = absl::make_unique<MapManager>("map");
  std::string map_file;
  bool show_disable_submap;
  node_handle_.param("pbstream_file_path", map_file,
                     std::string("/home/map/grid2d.pbstream"));
  node_handle_.param("show_disable_submap", show_disable_submap, true);
  map_manager_->LoadMap(map_file, show_disable_submap);

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          "/submap_list", 1);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          "/trajectory_node_list", 1);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          "/constraint_list", 1);

  submap_query_server_ = node_handle_.advertiseService(
      "submap_query", &NodeMain::HandleSubmapQuery, this);
  save_map_server_ = node_handle_.advertiseService(
      "/carto_map/save_map", &NodeMain::HandleSaveMap, this);
  optimize_submap_server_ = node_handle_.advertiseService(
      "/carto_map/optimize_submap", &NodeMain::HandleOptimizeSubmap, this);
  remove_submap_server_ = node_handle_.advertiseService(
      "/carto_map/remove_submap", &NodeMain::HandleRemoveSubmap, this);
  remove_trajectory_server_ = node_handle_.advertiseService(
      "/carto_map/remove_trajectory", &NodeMain::HandleRemoveTrajectory, this);
  compute_overlap_submap_server_ = node_handle_.advertiseService(
      "/carto_map/compute_overlap_submap",
      &NodeMain::HandleComputeOverlapSubmap, this);

  submap_list_pub_timer_ = node_handle_.createWallTimer(
      ::ros::WallDuration(0.3), &NodeMain::PublishSubmapList, this);
  trajectory_node_list_pub_timer_ = node_handle_.createWallTimer(
      ::ros::WallDuration(30e-3), &NodeMain::PublishTrajectoryNodeList, this);
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
  map_manager_->SaveMap(request.filename);
  return true;
}

bool NodeMain::HandleOptimizeSubmap(
    carto_map::OptimizeSubmap::Request& request,
    carto_map::OptimizeSubmap::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_manager_->HandleOptimizeSubmap(request, response);
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
