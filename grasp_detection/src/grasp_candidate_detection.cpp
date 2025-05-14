#include "grasp_detection/grasp_candidate_detection.h"


// Main function to detect grasp candidates 
std::vector<GraspCandidate> detectGraspCandidates(const PointCloudT::ConstPtr &cloud,
                                                  const std::vector<pcl::PointIndices> &edge_indices,
                                                  float max_gripper_width) 
{
    std::vector<GraspCandidate> candidates;


    // Debugging
    ROS_INFO("Processing %zu edge clusters for grasp detection", edge_indices.size());
    

    // Extract all edge points into a single cloud for easier processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_edge_points(new pcl::PointCloud<pcl::PointXYZ>);
    

    for (const auto &indices : edge_indices) 
    {
        for (const auto &i : indices.indices) 
        {
            all_edge_points->points.push_back(cloud->points[i]);
        }
    }
    

    if (all_edge_points->points.empty()) 
    {
        ROS_WARN("No edge points available for grasp detection");
        return candidates;
    }
    

    // Convert to unorganized point cloud (1D array)
    all_edge_points->width = all_edge_points->points.size();
    all_edge_points->height = 1;
    all_edge_points->is_dense = false;
    

    // Debugging
    ROS_INFO("Total edge points for grasp detection: %zu", all_edge_points->points.size());
    
    
    std::vector<GraspPointPair> potential_pairs;

    // Find all pairs within max_gripper_width
    for (size_t i = 0; i < all_edge_points->points.size(); i++) 
    {
        for (size_t j = i+1; j < all_edge_points->points.size(); j++) 
        {
            const auto &p1 = all_edge_points->points[i];
            const auto &p2 = all_edge_points->points[j];
    
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float dz = p1.z - p2.z;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Check if points are within gripper width and not too close
            if (dist <= max_gripper_width && dist > 0.01)   // 1cm minimum to avoid trivial pairs
            {
                potential_pairs.push_back({p1, p2, dist});
            }
        }
    }
    

    ROS_INFO("Found %zu potential grasp point pairs within gripper width", potential_pairs.size());
    

    // Sort pairs by distance from origin
    std::sort(potential_pairs.begin(), potential_pairs.end(), 
    [](const GraspPointPair &a, const GraspPointPair &b) 
    {
    float dist_a = std::sqrt((a.p1.x + a.p2.x)*(a.p1.x + a.p2.x) + 
                             (a.p1.y + a.p2.y)*(a.p1.y + a.p2.y) + 
                             (a.p1.z + a.p2.z)*(a.p1.z + a.p2.z))/4.0;
    
    float dist_b = std::sqrt((b.p1.x + b.p2.x)*(b.p1.x + b.p2.x) + 
                             (b.p1.y + b.p2.y)*(b.p1.y + b.p2.y) + 
                             (b.p1.z + b.p2.z)*(b.p1.z + b.p2.z))/4.0;
    
    return dist_a < dist_b;
    });
    

    // Take the best candidate
    if (!potential_pairs.empty()) 
    {
        // Use the closest pair as our grasp candidate
        const auto &best_pair = potential_pairs[0];
        
        GraspCandidate candidate;
        
        // Calculate grasp center as the midpoint between points
        candidate.grasp_center = Eigen::Vector3f((best_pair.p1.x + best_pair.p2.x) / 2.0f,
                                                 (best_pair.p1.y + best_pair.p2.y) / 2.0f,
                                                 (best_pair.p1.z + best_pair.p2.z) / 2.0f);
        
        // Calculate approach direction (normalized vector from p1 to p2)
        Eigen::Vector3f approach(best_pair.p2.x - best_pair.p1.x,
                                 best_pair.p2.y - best_pair.p1.y,
                                 best_pair.p2.z - best_pair.p1.z);

        candidate.approach_direction = approach.normalized();
        
        // Set width
        candidate.width = best_pair.distance;
        
        ROS_INFO("Found grasp candidate: center=[%.3f,%.3f,%.3f], width=%.3f",
        candidate.grasp_center[0], candidate.grasp_center[1], candidate.grasp_center[2],
        candidate.width);
        
        candidates.push_back(candidate);
    } 
    else 
    {
        ROS_WARN("No valid grasp candidates found within gripper width constraint (%.3f m)", 
                 max_gripper_width);
    }
    
    return candidates; 
}



// Transform local coordinates to global coordinates and publish the grasp pose
geometry_msgs::Pose publishGraspPose(const GraspCandidate& grasp, ros::Publisher& pose_pub) 
{
    // Create a pose in the HazCam frame
    geometry_msgs::PoseStamped local_pose;
    local_pose.header.frame_id = "haz_cam";
    local_pose.header.stamp = ros::Time::now();
    
    // Set position
    local_pose.pose.position.x = grasp.grasp_center[0];
    local_pose.pose.position.y = grasp.grasp_center[1];
    local_pose.pose.position.z = grasp.grasp_center[2];
    
    // Convert approach direction to orientation (quaternion)
    Eigen::Vector3f z_axis = grasp.approach_direction;
    Eigen::Vector3f x_axis;
    
    // Create a perpendicular vector for x-axis
    if (std::abs(z_axis[0]) < std::abs(z_axis[1]) && std::abs(z_axis[0]) < std::abs(z_axis[2])) 
    {
        x_axis = Eigen::Vector3f(0, -z_axis[2], z_axis[1]).normalized();
    } 
    else 
    {
        x_axis = Eigen::Vector3f(-z_axis[1], z_axis[0], 0).normalized();
    }

    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    
    // Create rotation matrix
    Eigen::Matrix3f rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    

    // Convert to quaternion
    Eigen::Quaternionf quaternion(rotation);
    local_pose.pose.orientation.x = quaternion.x();
    local_pose.pose.orientation.y = quaternion.y();
    local_pose.pose.orientation.z = quaternion.z();
    local_pose.pose.orientation.w = quaternion.w();
    

    geometry_msgs::PoseStamped world_pose;
    
    // Transform from haz_cam frame to world frame
    try 
    {
        // Set up the TF buffer and listener
        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);
        
        // Wait for the transform to be available
        if (tf_buffer.canTransform("world", "haz_cam", ros::Time(0), ros::Duration(1.0))) 
        {
            // Get the transform
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("world", 
                                                                                  "haz_cam", 
                                                                                  ros::Time(0));
            
            // Apply transform to our pose
            tf2::doTransform(local_pose, world_pose, transform);
            
            ROS_INFO("World pose: position [%.3f, %.3f, %.3f]", world_pose.pose.position.x,
                                                                world_pose.pose.position.y,
                                                                world_pose.pose.position.z);
        } 
        else 
        {
            ROS_WARN("Could not transform from 'haz_cam' to 'world', using local pose");
        }
    } 
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("Transform exception: %s", ex.what());
    }
    
    // Publish the transformed pose
    pose_pub.publish(world_pose.pose);
    
    return world_pose.pose;
}