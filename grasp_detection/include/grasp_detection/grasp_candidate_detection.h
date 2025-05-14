#ifndef GRASP_DETECTION_H
#define GRASP_DETECTION_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

struct GraspCandidate 
{
    Eigen::Vector3f grasp_center;
    Eigen::Vector3f approach_direction;
    float width;
};

struct GraspPointPair 
{
    pcl::PointXYZ p1;
    pcl::PointXYZ p2;
    float distance;
};

std::vector<GraspCandidate> detectGraspCandidates(
    const PointCloudT::ConstPtr &cloud,
    const std::vector<pcl::PointIndices> &edge_indices,
    float max_gripper_width);


geometry_msgs::Pose publishGraspPose(const GraspCandidate& grasp, ros::Publisher& pose_pub);

#endif