#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>


using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

visualization_msgs::MarkerArray createEdgeMarkers(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                                  const PointCloudT::Ptr &cloud,
                                                  const std::vector<pcl::PointIndices> &edge_indices);

void visualizeGraspPose(const geometry_msgs::Pose& pose, const std::string& frame_id, ros::Publisher& marker_pub);

#endif
