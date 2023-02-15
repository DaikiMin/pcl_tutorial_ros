#include "pcl_tutorial_ros/basic_point_cloud_handle.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pch {
    BasicPointCloudHandle::BasicPointCloudHandle(){
        // set Default value :
        setPassThroughParameters( "z", 0.1, 1.0);
        setVoxelGridParameter( 0.01 );
        setClusteringParameters( 0.05, 1, 1000 );
        tree_ .reset ( new pcl::search::KdTree<PointT>() );
    }
    // Transform a coordinate frame :
    bool BasicPointCloudHandle::transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud ) {
        PointCloud cloud_src;
        pcl::fromROSMsg<PointT>( *input_cloud, cloud_src );
        if (target_frame.empty() == false ){
            try {
                tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
                output_cloud->header.frame_id = target_frame;
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return false;
            }
        } else ROS_ERROR("Please set the target frame.");
        return true;
    }
    // Extract point cloud within set range :
    bool BasicPointCloudHandle::passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
        try {
            PointCloud::Ptr tmp ( new PointCloud() );
            pass_.setInputCloud( input_cloud );
            pass_.filter( *tmp );
            *output_cloud = *tmp;
            output_cloud->header.frame_id = input_cloud->header.frame_id;
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Downsample a point cloud :
    bool BasicPointCloudHandle::voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) { 
        try {
            PointCloud::Ptr tmp ( new PointCloud() );
            voxel_.setInputCloud( input_cloud ); 
            voxel_.filter( *tmp );
            *output_cloud = *tmp;
            output_cloud->header.frame_id = input_cloud->header.frame_id;
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // clustering extraction :
    bool BasicPointCloudHandle::euclideanClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {  
        try {
            tree_->setInputCloud( input_cloud );
            ec_.setInputCloud( input_cloud );
            ec_.extract( *output_indices );
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Extract specified Indices from point cloud :
    bool BasicPointCloudHandle::extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative ) {
        try {
            PointCloud::Ptr tmp ( new PointCloud() );
            extract_.setInputCloud( input_cloud );
            extract_.setIndices( indices );
            extract_.setNegative( negative );
            extract_.filter( *tmp );
            *output_cloud = *tmp;
            output_cloud->header.frame_id = input_cloud->header.frame_id;
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
}