#include "grasp_detection/edge_detection.h"


// This function detects edges using organized edge detection from normals
std::vector<pcl::PointIndices> edgeDetection(const PointCloudT::Ptr &cloud)
{
    // Compute normals for the organized point cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setKSearch(10);
    normal_estimation.compute(*normals);
    
    // Initialize the edge detector and label cloud
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> edge_indices;

    pcl::OrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label> edge_detector;
    edge_detector.setInputCloud(cloud);
    edge_detector.setInputNormals(normals);

    // Choose which types of edges to detect
    edge_detector.setEdgeType(pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_OCCLUDING |
                              pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_OCCLUDED |
                              pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_NAN_BOUNDARY);

    edge_detector.compute(*labels, edge_indices);

    return edge_indices;
}
