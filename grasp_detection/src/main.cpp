#include <pcl_conversions/pcl_conversions.h>

#include "grasp_detection/edge_detection.h"
#include "grasp_detection/visualization.h"
#include "grasp_detection/grasp_candidate_detection.h"


ros::Publisher edge_marker_pub;
ros::Publisher grasp_pose_pub;
ros::Publisher grasp_marker_pub;

ros::Subscriber point_cloud_sub; 


// Callback that processes the incoming point cloud data
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    ROS_INFO("Received PointCloud2 data");

    // Convert the ROS PointCloud2 message to a PCL point cloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Perform edge detection
    std::vector<pcl::PointIndices> edge_indices = edgeDetection(cloud);

    // Create and publish visualization markers
    visualization_msgs::MarkerArray marker_array = createEdgeMarkers(cloud_msg, cloud, edge_indices);
    edge_marker_pub.publish(marker_array);

    // Detect grasp candidates - maximum gripper width: 8.4 cm
    float max_gripper_width = 0.084;
    std::vector<GraspCandidate> candidates = detectGraspCandidates(cloud, edge_indices, max_gripper_width);

    if (!candidates.empty()) 
    {
        // Publish the first grasp candidate's pose
        geometry_msgs::Pose grasp_pose = publishGraspPose(candidates[0], grasp_pose_pub);
        ROS_INFO("Published grasp pose at (x: %.2f, y: %.2f, z: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f)",
                 grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, grasp_pose.orientation.x,
                 grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
        
        // Visualize the grasp pose in RViz
        visualizeGraspPose(grasp_pose, "world", grasp_marker_pub);
        
        
        // Unsubscribe from point cloud topic - we don't need more data
        point_cloud_sub.shutdown();
        ROS_INFO("Grasp pose published once. Unsubscribing from point cloud topic.");
    }
    else
    {
        ROS_WARN("No grasp candidate detected!");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_detection_node");
    ros::NodeHandle nh;

    // Subscribe to the point cloud topic
    point_cloud_sub = nh.subscribe("/hw/depth_haz/points", 1, cloudCallback);

    // Publisher for edge visualization markers
    edge_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("edge_markers", 1);

    // Publisher for the grasp pose
    grasp_pose_pub = nh.advertise<geometry_msgs::Pose>("grasp_pose", 1);

    // Publisher for grasp visualization markers
    grasp_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("grasp_marker", 1);

    ROS_INFO("Grasp detection node started. Waiting for point cloud data...");
    ros::spin();
    return 0;
}