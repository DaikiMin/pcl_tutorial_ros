#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>            
#include <pcl_ros/transforms.h>             
#include <pcl/point_types.h>                
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pch {
    class BasicPointCloudHandle {
    private :
        /* tf */
        tf::TransformListener tf_listener_;
    public :
        BasicPointCloudHandle(){};

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
            } catch (tf::TransformException &ex) {
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
        std::string target_frame_;
        pch::BasicPointCloudHandle pch_;
        
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
            PointCloud::Ptr cloud (new PointCloud());
            pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud );
            pub_cloud_.publish(cloud);
            ROS_INFO("cloud points size = %zu\n", cloud->points.size());
        }

    public:
        PointcloudSsubscriber(): nh_(), pnh_("~") {
            std::string topic_name = pnh_.param<std::string>( "topic_name", "/sensor_data" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            
            ROS_INFO("target_frame = '%s'", target_frame_.c_str());
            ROS_INFO("topic_name = '%s'", topic_name.c_str());

            pub_cloud_ = nh_.advertise<PointCloud>("/transformed_cloud", 1);
            sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudSsubscriber::cbPoints, this);
        }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pointcloud_subscriber");

  PointcloudSsubscriber pointcloud_subscriber;
  ros::spin();
}
