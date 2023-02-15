#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <random>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <visualization_msgs/MarkerArray.h>
#include "pcl_tutorial_ros/basic_point_cloud_handle.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<NormalT> NomalCloud;


namespace rgs {
    class RegionGrowingSegmentation {
        private : 
            pcl::NormalEstimation<PointT, NormalT> ne_;
            pcl::RegionGrowing<PointT, NormalT> reg_; 
            pcl::search::KdTree<PointT>::Ptr tree_;
        public :
            RegionGrowingSegmentation() {
                tree_ .reset ( new pcl::search::KdTree<PointT>() );
                ne_.setSearchMethod (tree_);
                ne_.setKSearch (50);
                reg_.setMinClusterSize (5);
                reg_.setMaxClusterSize (5000);
                reg_.setSearchMethod (tree_);
                reg_.setNumberOfNeighbours (30);
                reg_.setSmoothnessThreshold (8.0 / 180.0 * M_PI);
                reg_.setCurvatureThreshold (1.0);
            }

            bool segment( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {
                NomalCloud::Ptr normals (new NomalCloud());
                std::vector <pcl::PointIndices> clusters;
                ne_.setInputCloud (input_cloud);
                ne_.compute (*normals);
                reg_.setInputCloud (input_cloud);
                reg_.setInputNormals (normals);
                reg_.extract (clusters); 
                *output_indices = clusters;
            }
    };
}


class PointcloudSsubscriber {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        ros::Publisher pub_marker_; 
        std::string target_frame_;
        pch::BasicPointCloudHandle pch_;
        rgs::RegionGrowingSegmentation rgs_;

        void displayMarker( const PointCloud::Ptr input_cloud, const std::vector<pcl::PointIndices>& input_indices ) const {
            visualization_msgs::MarkerArray marker_array;
            std::random_device rnd;
            std::mt19937 mt(rnd()); 
            std::uniform_real_distribution<> rand1(0.0, 1.0); 
            int marker_id = 0;
            float r,g,b;
            for( auto& cluster : input_indices ) {
                r = rand1(mt);
                g = rand1(mt);
                b = rand1(mt);
                for ( auto& i : cluster.indices ) {
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = target_frame_;
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "cluster";
                    marker.id = marker_id;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;

                    marker.pose.position.x = input_cloud->points[i].x;
                    marker.pose.position.y = input_cloud->points[i].y;
                    marker.pose.position.z = input_cloud->points[i].z;

                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;

                    marker.scale.x = 0.02;
                    marker.scale.y = 0.02;
                    marker.scale.z = 0.02;

                    marker.color.r = r;
                    marker.color.g = g;
                    marker.color.b = b;
                    marker.color.a = 1.0f;

                    marker.lifetime = ros::Duration();
                    marker_id++;
                    marker_array.markers.push_back( marker );
                }
            }
            pub_marker_.publish( marker_array );
            return;
        }
        
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            std::vector <pcl::PointIndices> clusters;
            pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud );
            pch_.passThrough( cloud, cloud );
            pch_.voxelGrid( cloud, cloud );
            rgs_.segment( cloud, &clusters );
            displayMarker( cloud, clusters );
            pub_cloud_.publish(cloud);
            ROS_INFO("clusters count : %zu", clusters.size() );
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/sensor_data" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            pch_.setPassThroughParameters( "z", 0.1, 1.3 );
            pch_.setVoxelGridParameter( 0.025 );

            ROS_INFO("target_frame = '%s'", target_frame_.c_str());
            ROS_INFO("topic_name = '%s'", topic_name.c_str());

            pub_cloud_ = nh_.advertise<PointCloud>("/cloud", 1);
            pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/marker", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
        }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "region_growing_segmentation");

  PointcloudSsubscriber pointcloud_subscriber;
  ros::spin();
}
