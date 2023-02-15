#include <ros/ros.h>                            
#include <iostream>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
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

        public:
            BasicPointCloudHandle(){
                // set Default value :
                setPassThroughParameters( "z", 0.1, 1.0);
                setVoxelGridParameter( 0.01 );
                setClusteringParameters( 0.05, 1, 1000 );
                tree_ .reset ( new pcl::search::KdTree<PointT>() );
            }
            // set passThrough Parameters :
            void setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
                pass_.setFilterFieldName( axis );
                pass_.setFilterLimits( limit_min, limit_max);
            }
            // Seting VoxelGrid Parameters :
            void setVoxelGridParameter( const float leaf_size ) { 
                voxel_.setLeafSize( leaf_size, leaf_size, leaf_size );
            }
            void setClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
                ec_.setClusterTolerance( tolerance );
                ec_.setMinClusterSize( min_size );
                ec_.setMaxClusterSize( max_size );
                ec_.setSearchMethod( tree_ );
            }
            // Transform a coordinate frame :
            bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud ) {
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
            bool passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
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
            bool voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) { 
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
            bool euclideanClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {  
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
    };
}


class PointcloudSsubscriber {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        ros::Publisher pub_cloud_pass_;
        ros::Publisher pub_cloud_voxel_;
        ros::Publisher pub_tgt_; 
        ros::Publisher pub_marker_; 
        std::string target_frame_;
        pch::BasicPointCloudHandle pch_;
        std::vector<double> cluster_size_min_;
		std::vector<double> cluster_size_max_;

        visualization_msgs::Marker makeMarker( const std::string &marker_ns,int marker_id, 
            const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt ) const {
            visualization_msgs::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = marker_ns;
            marker.id = marker_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
            marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
            marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = max_pt.x() - min_pt.x();
            marker.scale.y = max_pt.y() - min_pt.y();
            marker.scale.z = max_pt.z() - min_pt.z();

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3f;

            marker.lifetime = ros::Duration(0.3);
            return marker;
        }

        bool detectNearestCluster ( const PointCloud::Ptr input_cloud, const std::vector<pcl::PointIndices>& input_indices, 
            PointCloud::Ptr output_cloud, visualization_msgs::MarkerArray* output_marker ) {
            PointCloud::Ptr cloud_target (new PointCloud());
            pcl::PointIndices::Ptr cluster_target(new pcl::PointIndices);
            visualization_msgs::MarkerArray marker_array;

            double tgt_dist = DBL_MAX;
            int marker_id = 0;
            int target_index = -1;
            for ( auto& cluster : input_indices ) {
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D( *input_cloud , cluster, min_pt, max_pt);
                Eigen::Vector4f cluster_size = max_pt - min_pt;
                visualization_msgs::Marker marker = makeMarker( "cluster", marker_id, min_pt, max_pt );
                marker_id++;
                bool is_ok = true;
                if ( cluster_size.x() == 0.0 && cluster_size.y() == 0.0 && cluster_size.z() == 0.0 ) is_ok = false;
                if ( cluster_size.x() < cluster_size_min_[0] || cluster_size.x() > cluster_size_max_[0] ) is_ok = false;
                if ( cluster_size.y() < cluster_size_min_[1] || cluster_size.y() > cluster_size_max_[1] ) is_ok = false;
                if ( cluster_size.z() < cluster_size_min_[2] || cluster_size.z() > cluster_size_max_[2] ) is_ok = false;
                if( is_ok ) {
                    marker.ns = "ok_cluster";
                    marker.color.r = 1.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f;
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid( *input_cloud, cluster, centroid );
                    double dist = std::hypotf( centroid.x(), centroid.y() );
                    if( dist < tgt_dist ) {
                        tgt_dist = dist;
                        *cluster_target = cluster;
                        target_index = marker_array.markers.size();
                    }
                }
                marker_array.markers.push_back(marker);
            }
            if ( target_index != -1 ) {
                marker_array.markers[target_index].ns = "target_cluster";
                marker_array.markers[target_index].color.r = 1.0f;
                marker_array.markers[target_index].color.g = 0.0f;
                marker_array.markers[target_index].color.b = 1.0f;
                for ( auto& i : cluster_target->indices ) cloud_target->points.push_back ( input_cloud->points[i] ); 
                cloud_target->header.frame_id = target_frame_;
            }
            *output_cloud = *cloud_target;
            *output_marker = marker_array;
            return (target_index != -1) ? true : false;
        }
        
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            PointCloud::Ptr cloud_pass (new PointCloud());
            PointCloud::Ptr cloud_voxel (new PointCloud());
            PointCloud::Ptr cloud_target (new PointCloud());
            std::vector<pcl::PointIndices> cluster_indices;
            visualization_msgs::MarkerArray marker_array;

            pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud );
            pch_.passThrough( cloud, cloud_pass );
            pch_.voxelGrid( cloud_pass, cloud_voxel );
            pch_.euclideanClusterExtraction( cloud_voxel , &cluster_indices );
            detectNearestCluster( cloud_voxel, cluster_indices, cloud_target, &marker_array );

            pub_cloud_.publish(cloud);
            pub_cloud_pass_.publish(cloud_pass);
            pub_cloud_voxel_.publish(cloud_voxel);
            pub_tgt_.publish( cloud_target );
            pub_marker_.publish( marker_array );

            ROS_INFO("\ncloud points size = %zu\npass points size = %zu\nvoxel points size = %zu\ntarget points size = %zu\n", 
                cloud->points.size(), cloud_pass->points.size(), cloud_voxel->points.size(), cloud_target->points.size());
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/sensor_data" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            std::string pass_axis = pnh_.param<std::string>( "pass_axis", "z" );
            float pass_min = pnh_.param<float>( "pass_min_range", 0.2 );
            float pass_max = pnh_.param<float>( "pass_max_range", 1.0 );
            pch_.setPassThroughParameters( pass_axis, pass_min, pass_max );
            float leaf_size = pnh_.param<float>( "voxel_leaf_size", 0.03 );
            pch_.setVoxelGridParameter( leaf_size );
            double tolerance = pnh_.param<double>( "clustering_tolerance", 0.05 );
            int min_size = pnh_.param<int>( "clustering_points_min_size", 5 );
            int max_size = pnh_.param<int>( "clustering_points_max_size", 5000 );
            pch_.setClusteringParameters( tolerance, min_size, max_size );
            cluster_size_min_ = pnh_.param<std::vector<double>>( "cluster_size_min", {0.1, 0.1, 0.1} );
            cluster_size_max_ = pnh_.param<std::vector<double>>( "cluster_size_max", {1.5, 1.5, 1.5} );
            

            std::cout << "========================================"
                        << "\n[ Parameters ]" 
                        << "\n  * topic_name : " << topic_name
                        << "\n  * target_frame : " << target_frame_
                        << "\n  * pass_axis : " << pass_axis
                        << "\n  * pass_min_range : " << pass_min
                        << "\n  * pass_max_range : " << pass_max
                        << "\n  * voxel_leaf_size : " << leaf_size
                        << "\n  * clustering_tolerance : " << tolerance
                        << "\n  * clustering_points_min_size : " << min_size
                        << "\n  * clustering_points_max_size : " << max_size
                        << "\n  * cluster_size_min : " << cluster_size_min_[0] << ", " << cluster_size_min_[1] << ", " << cluster_size_min_[2]
                        << "\n  * cluster_size_max : " << cluster_size_max_[0] << ", " << cluster_size_max_[1] << ", " << cluster_size_max_[2]
                        << "\n========================================"
                        << std::endl;

            pub_cloud_ = nh_.advertise<PointCloud>("/transformed_cloud", 1);
            pub_cloud_pass_ = nh_.advertise<PointCloud>("/pass_cloud", 1);
            pub_cloud_voxel_ = nh_.advertise<PointCloud>("/voxel_cloud", 1);
            pub_tgt_ = nh_.advertise<PointCloud>("/target_cloud", 1);
            pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/marker", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
        }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "clustering");

  PointcloudSsubscriber pointcloud_subscriber;
  ros::spin();
}
