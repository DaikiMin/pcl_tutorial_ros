#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "pcl_tutorial_ros/basic_point_cloud_handle.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pd {
    class PlaneDetection {
        private :
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::ExtractIndices<PointT> extract; 
        public :
            PlaneDetection(void){
                seg.setOptimizeCoefficients (true);
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                setPlaneDetectionParameter( 0.01, 0.95 );
            }
            bool setPlaneDetectionParameter( const double threshold, const double probability ){
                try{ 
                    seg.setDistanceThreshold (threshold);
                    seg.setProbability(probability);
                    return true;
                } catch ( std::exception& ex ) {
                    ROS_ERROR("%s", ex.what());
                    return false;
                }
            }
            bool detectPlane( const PointCloud::Ptr input_cloud, PointCloud::Ptr plane_cloud ){
                try{ 
                    PointCloud::Ptr tmp (new PointCloud());
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    seg.setInputCloud (input_cloud);
                    seg.segment (*inliers, *coefficients);
                    extract.setInputCloud(input_cloud); 
                    extract.setIndices(inliers);
                    extract.setNegative(false);
                    extract.filter(*tmp);
                    tmp->header.frame_id = input_cloud->header.frame_id;
                    *plane_cloud = *tmp;
                    return true;
                } catch ( std::exception& ex ) {
                    ROS_ERROR("%s", ex.what());
                    return false;
                }
            }
            bool removePlane( const PointCloud::Ptr input_cloud, PointCloud::Ptr remove_cloud ){
                try{ 
                    PointCloud::Ptr tmp (new PointCloud());
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    seg.setInputCloud (input_cloud);
                    seg.segment (*inliers, *coefficients);
                    extract.setInputCloud(input_cloud); 
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*tmp);
                    tmp->header.frame_id = input_cloud->header.frame_id;
                    *remove_cloud = *tmp;
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
        ros::Publisher pub_cloud_plane_;
        ros::Publisher pub_cloud_remove_;
        std::string target_frame_;
        pch::BasicPointCloudHandle pch_;
        pd::PlaneDetection pd_;
        
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            PointCloud::Ptr cloud_plane (new PointCloud());
            PointCloud::Ptr cloud_remove (new PointCloud());
            pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud );
            pch_.voxelGrid( cloud, cloud );
            pd_.detectPlane( cloud, cloud_plane );
            pd_.removePlane( cloud, cloud_remove );
            pub_cloud_.publish(cloud);
            pub_cloud_plane_.publish(cloud_plane);
            pub_cloud_remove_.publish( cloud_remove );
            ROS_INFO("\ncloud points size = %zu\nplane points size = %zu\nremove points size = %zu\n", 
                cloud->points.size(), cloud_plane->points.size(), cloud_remove->points.size());
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/sensor_data" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            float leaf_size = pnh_.param<float>( "voxel_leaf_size", 0.03 );
            pch_.setVoxelGridParameter( leaf_size );
            float threshold = pnh_.param<float>( "segment_distance_threshold", 0.03 );
            float probability = pnh_.param<float>( "segment_probability", 0.90 );
            pd_.setPlaneDetectionParameter( threshold, probability );
            std::cout << "========================================"
                        << "\n[ Parameters ]" 
                        << "\n  * topic_name : " << topic_name
                        << "\n  * target_frame : " << target_frame_
                        << "\n  * voxel_leaf_size : " << leaf_size
                        << "\n  * segment_distance_threshold : " << threshold
                        << "\n  * segment_probability : " << probability
                        << "\n========================================"
                        << std::endl;
            pub_cloud_ = nh_.advertise<PointCloud>("/cloud", 1);
            pub_cloud_plane_ = nh_.advertise<PointCloud>("/cloud_plane", 1);
            pub_cloud_remove_ = nh_.advertise<PointCloud>("/cloud_remove", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
        }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "plane_detection");

  PointcloudSsubscriber pointcloud_subscriber;
  ros::spin();
}
