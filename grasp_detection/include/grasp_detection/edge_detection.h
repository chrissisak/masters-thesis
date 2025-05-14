#ifndef EDGE_PROCESSING_H
#define EDGE_PROCESSING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <vector>


using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;


std::vector<pcl::PointIndices> edgeDetection(const PointCloudT::Ptr &cloud);

#endif
