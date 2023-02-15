#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>            
#include <pcl_ros/transforms.h>             
#include <pcl/point_types.h>   
#include <pcl/filters/passthrough.h>              
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pch {
    class BasicPointCloudHandle {
    private :
        /* tf */
        tf::TransformListener tf_listener_;
        /* Point Cloud Library */
        pcl::PassThrough<PointT> pass_;
    public :
        BasicPointCloudHandle(){};
        // set passThrough Parameters :
        void setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
            pass_.setFilterFieldName( axis );
            pass_.setFilterLimits( limit_min, limit_max);
        }
        // Transform a coordinate frame :
        bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud ){
            PointCloud cloud_src;
            pcl::fromROSMsg<PointT>( *input_cloud, cloud_src );
            if (target_frame.empty() ) {
                ROS_ERROR("Please set the target frame.");
                return true;
            }
            try {
                // transform frame :
                tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
                output_cloud->header.frame_id = target_frame;
                return true;
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return false;
            }
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
    };
}


class PointcloudSsubscriber {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_cloud_;
        ros::Publisher pub_cloud_pass_;
        std::string target_frame_;
        pch::BasicPointCloudHandle pch_;
        
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            PointCloud::Ptr cloud_pass (new PointCloud());
            pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud );
            pch_.passThrough( cloud, cloud_pass );
            pub_cloud_.publish(cloud);
            pub_cloud_pass_.publish(cloud_pass);
            ROS_INFO("\ncloud points size = %zu\npass points size = %zu\n", cloud->points.size(), cloud_pass->points.size());
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/sensor_data" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            std::string pass_axis = pnh_.param<std::string>( "pass_axis", "z" );
            float pass_min = pnh_.param<float>( "pass_min_range", 0.2 );
            float pass_max = pnh_.param<float>( "pass_max_range", 1.0 );
            pch_.setPassThroughParameters( pass_axis, pass_min, pass_max );

            std::cout << "========================================"
                        << "\n[ Parameters ]" 
                        << "\n  * topic_name : " << topic_name
                        << "\n  * target_frame : " << target_frame_
                        << "\n  * pass_axis : " << pass_axis
                        << "\n  * pass_min : " << pass_min
                        << "\n  * pass_max : " << pass_max
                        << "\n========================================"
                        << std::endl;

            pub_cloud_ = nh_.advertise<PointCloud>("/transformed_cloud", 1);
            pub_cloud_pass_ = nh_.advertise<PointCloud>("/pass_cloud", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
        }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "passthrough");

  PointcloudSsubscriber pointcloud_subscriber;
  ros::spin();
}
