#include <ros/ros.h>                            
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_tutorial_ros/basic_point_cloud_handle.h"
#include "pcl_tutorial_ros/point_cloud_tracker.h" // <- pcl/tracking class

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointcloudSsubscriber {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        ros::Publisher pub_tgt_; 
        ros::Publisher pub_particles_;
        ros::Publisher pub_marker_;
        std::string target_frame_;
        pch::BasicPointCloudHandle pch_;
        pct::PointCloudTracker pct_;
        std::vector<double> cluster_size_min_;
		std::vector<double> cluster_size_max_;
        bool exists_tgt_;

        void displayCentroidMarker ( const geometry_msgs::Point &target_pt ) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "target_pose";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position = target_pt;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.25;
            marker.scale.y = 0.25;
            marker.scale.z = 0.25;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;

            marker.lifetime = ros::Duration(0.3);
            pub_marker_.publish( marker );
            return;
        }

        bool detectNearestCluster ( const PointCloud::Ptr input_cloud, const std::vector<pcl::PointIndices>& input_indices, 
            PointCloud::Ptr output_cloud, geometry_msgs::Point* output_centroid ) {
            pcl::PointIndices::Ptr cluster_target(new pcl::PointIndices);
            Eigen::Vector4f centroid_target;
            double tgt_dist = DBL_MAX;
            bool is_target = false;
            for ( auto& cluster : input_indices ) {
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D( *input_cloud , cluster, min_pt, max_pt);
                Eigen::Vector4f cluster_size = max_pt - min_pt;
                if ( cluster_size.x() == 0.0 && cluster_size.y() == 0.0 && cluster_size.z() == 0.0 ) continue;
                if ( cluster_size.x() < cluster_size_min_[0] || cluster_size.x() > cluster_size_max_[0] ) continue;
                if ( cluster_size.y() < cluster_size_min_[1] || cluster_size.y() > cluster_size_max_[1] ) continue;
                if ( cluster_size.z() < cluster_size_min_[2] || cluster_size.z() > cluster_size_max_[2] ) continue;
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid( *input_cloud, cluster, centroid );
                double dist = std::hypotf( centroid.x(), centroid.y() );
                if( dist < tgt_dist ) {
                    tgt_dist = dist;
                    *cluster_target = cluster;
                    centroid_target = centroid;
                    is_target = true;
                }
            }
            if ( is_target ) {
                pch_.extractIndices( input_cloud, output_cloud, cluster_target, false ); 
                output_cloud->header.frame_id = target_frame_;
                output_centroid->x = centroid_target.x();
                output_centroid->y = centroid_target.y();
                output_centroid->z = centroid_target.z();
            }
            return is_target;
        }
        
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            PointCloud::Ptr cloud_target (new PointCloud());
            PointCloud::Ptr cloud_particles (new PointCloud());
            geometry_msgs::Point target_centroid;
            std::vector<pcl::PointIndices> cluster_indices;

            pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud );
            pch_.passThrough( cloud, cloud );
            pch_.voxelGrid( cloud, cloud );

            if ( !exists_tgt_ ) {
                pch_.euclideanClusterExtraction( cloud , &cluster_indices );
                exists_tgt_ = detectNearestCluster( cloud, cluster_indices, cloud_target, &target_centroid );
                pct_.setTrackTarget( cloud_target, target_centroid );
            } else {
                exists_tgt_ = pct_.getTrackResult( cloud, cloud_target, &target_centroid );
                pct_.getParticles( cloud_particles );
                cloud_particles->header.frame_id = target_frame_;
                pub_particles_.publish( cloud_particles );
            }
            pub_cloud_.publish(cloud);
            pub_tgt_.publish( cloud_target );
            displayCentroidMarker( target_centroid );
            ROS_INFO("\n[ Track Target Pose ]\nx : %.3f\ny : %.3f\nz : %.3f\n",
                target_centroid.x, target_centroid.y, target_centroid.z );
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
            pub_tgt_ = nh_.advertise<PointCloud>("/target_cloud", 1);
            pub_particles_ = nh_.advertise<PointCloud>("/particles_cloud", 1);
            pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/centroid_marker", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
            exists_tgt_ = false;
        }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tracker");
  PointcloudSsubscriber pointcloud_subscriber;
  ros::spin();
}
