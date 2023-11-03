#include "cartographer/transform/transform.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

geometry_msgs::Pose ToGeometryMsgPose(const ::cartographer::transform::Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}












