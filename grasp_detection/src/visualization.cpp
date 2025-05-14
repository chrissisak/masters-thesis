#include "grasp_detection/visualization.h"


// This function creates a MarkerArray for visualizing edge points in RViz
visualization_msgs::MarkerArray createEdgeMarkers(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                                  const PointCloudT::Ptr &cloud,
                                                  const std::vector<pcl::PointIndices> &edge_indices)
{
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  
  // Iterate through each edge cluster and create a marker for it
  for (const auto &indices : edge_indices)
    {
    visualization_msgs::Marker marker;
    marker.header = cloud_msg->header;
    marker.ns = "edges";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.005; // Point size
    marker.scale.y = 0.005;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Iterate through the points in the current edge cluster and add them to the marker
    for (const auto &index : indices.indices)
    {
        const auto &point = cloud->points[index];
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        // Only include points within 0.75 meters
        if (distance <= 0.75)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            marker.points.push_back(p);
        }
    }

    if (!marker.points.empty())
    {
      // Add the marker to the marker array
      marker_array.markers.push_back(marker);
    }
  }

  return marker_array;
}



void visualizeGraspPose(const geometry_msgs::Pose& pose, const std::string& frame_id, ros::Publisher& marker_pub) 
{
  visualization_msgs::MarkerArray marker_array;
  
  ros::Time now = ros::Time::now();
  
  // 1. Create a sphere to mark the grasp position
  visualization_msgs::Marker position_marker;
  position_marker.header.frame_id = frame_id;
  position_marker.header.stamp = now;
  position_marker.ns = "grasp_marker";
  position_marker.id = 0;
  position_marker.type = visualization_msgs::Marker::SPHERE;
  position_marker.action = visualization_msgs::Marker::ADD;
  position_marker.pose = pose;
  position_marker.scale.x = 0.01; // 1 cm diameter sphere
  position_marker.scale.y = 0.01;
  position_marker.scale.z = 0.01;
  position_marker.color.r = 1.0;  // Red
  position_marker.color.g = 0.0;
  position_marker.color.b = 0.0;
  position_marker.color.a = 0.8;
  marker_array.markers.push_back(position_marker);
  

  // 2. Create arrows to represent the axes of the grasp pose
  tf::Quaternion quat(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w
  );
  
  // Create standard basis vectors
  tf::Vector3 x_basis(1, 0, 0);
  tf::Vector3 y_basis(0, 1, 0);
  tf::Vector3 z_basis(0, 0, 1);
  
  // Apply rotation to get transformed axes
  tf::Transform transform(quat);
  tf::Vector3 x_axis = transform * x_basis;
  tf::Vector3 y_axis = transform * y_basis;
  tf::Vector3 z_axis = transform * z_basis;
  
  
  // X-Axis (Red)
  visualization_msgs::Marker x_marker;
  x_marker.header.frame_id = frame_id;
  x_marker.header.stamp = now;
  x_marker.ns = "grasp_marker";
  x_marker.id = 1;
  x_marker.type = visualization_msgs::Marker::ARROW;
  x_marker.action = visualization_msgs::Marker::ADD;

  // Set start point at the grasp position
  geometry_msgs::Point start;
  start.x = pose.position.x;
  start.y = pose.position.y;
  start.z = pose.position.z;
  x_marker.pose.orientation.w = 1.0;
  x_marker.points.push_back(start);
  
  // Set end point 10 cm along the x axis
  geometry_msgs::Point x_end;
  x_end.x = start.x + x_axis.x() * 0.1;
  x_end.y = start.y + x_axis.y() * 0.1;
  x_end.z = start.z + x_axis.z() * 0.1;
  x_marker.points.push_back(x_end);
  
  // Set marker appearance (size and color)
  x_marker.scale.x = 0.01;
  x_marker.scale.y = 0.02;
  x_marker.scale.z = 0.03;
  x_marker.color.r = 1.0;
  x_marker.color.g = 0.0;
  x_marker.color.b = 0.0;
  x_marker.color.a = 1.0;
  marker_array.markers.push_back(x_marker);
  
  // Y-Axis (Green)
  // Use the same properties as the x_marker for y-axis
  visualization_msgs::Marker y_marker = x_marker;
  y_marker.id = 2;
  y_marker.color.r = 0.0;
  y_marker.color.g = 1.0;
  y_marker.color.b = 0.0;
  y_marker.points.clear();
  y_marker.points.push_back(start);
  
  geometry_msgs::Point y_end;
  y_end.x = start.x + y_axis.x() * 0.1;
  y_end.y = start.y + y_axis.y() * 0.1;
  y_end.z = start.z + y_axis.z() * 0.1;
  y_marker.points.push_back(y_end);
  marker_array.markers.push_back(y_marker);
  

  // Z-Axis (Blue)
  // Use the same properties as the x_marker for z-axis
  visualization_msgs::Marker z_marker = x_marker;
  z_marker.id = 3;
  z_marker.color.r = 0.0;
  z_marker.color.g = 0.0;
  z_marker.color.b = 1.0;
  z_marker.points.clear();
  z_marker.points.push_back(start);
  
  geometry_msgs::Point z_end;
  z_end.x = start.x + z_axis.x() * 0.1; 
  z_end.y = start.y + z_axis.y() * 0.1;
  z_end.z = start.z + z_axis.z() * 0.1;
  z_marker.points.push_back(z_end);
  marker_array.markers.push_back(z_marker);
  
  // Publish the marker array
  marker_pub.publish(marker_array);
  ROS_INFO("Published grasp visualization markers");
}