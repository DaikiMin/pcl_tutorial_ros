#ifndef BASIC_POINT_CLOUD_HANDLE
#define BASIC_POINT_CLOUD_HANDLE

#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pch {
    class BasicPointCloudHandle {
        private:
            /* tf */
            tf::TransformListener tf_listener_;
            /* Point Cloud Library */
            pcl::PassThrough<PointT> pass_;
            pcl::VoxelGrid<PointT> voxel_;
            pcl::search::KdTree<PointT>::Ptr tree_;
            pcl::EuclideanClusterExtraction<PointT> ec_;
            pcl::ExtractIndices<PointT> extract_;

        public:
            BasicPointCloudHandle();
            // set passThrough Parameters :
            void setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max );
            // Seting VoxelGrid Parameters :
            void setVoxelGridParameter( const float leaf_size );
            void setClusteringParameters ( const float tolerance, const int min_size, const int max_size );
            // Transform a coordinate frame :
            bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud );
            // Extract point cloud within set range :
            bool passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            // Downsample a point cloud :
            bool voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            // clustering extraction :
            bool euclideanClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
            // Extract specified Indices from point cloud :
            bool extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative );
    };

    // set passThrough Parameters :
    inline void BasicPointCloudHandle::setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
        pass_.setFilterFieldName( axis );
        pass_.setFilterLimits( limit_min, limit_max);
    }
    // Seting VoxelGrid Parameters :
    inline void BasicPointCloudHandle::setVoxelGridParameter( const float leaf_size ) { 
        voxel_.setLeafSize( leaf_size, leaf_size, leaf_size );
    }
    inline void BasicPointCloudHandle::setClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
        ec_.setClusterTolerance( tolerance );
        ec_.setMinClusterSize( min_size );
        ec_.setMaxClusterSize( max_size );
        ec_.setSearchMethod( tree_ );
    }
}
#endif