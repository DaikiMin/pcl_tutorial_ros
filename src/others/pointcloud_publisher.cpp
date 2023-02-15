#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointcloudPublisherNode
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_cloud_sensor_;
        ros::Publisher pub_cloud_;
        std::string data_path_;

    public:
        PointcloudPublisherNode() : nh_(), pnh_("~") {
            pub_cloud_sensor_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor_data", 1);
            pub_cloud_ = nh_.advertise<PointCloud>("/PointCloud", 1);

            ros::param::get("data_path", data_path_);           
            
            std::cout << "====================\nLoad Data" << std::endl;
            std::cout << "data_path = "  << data_path_ << std::endl;
            std::cout << "====================\n" << std::endl;
        }

        int main(void){
            PointCloud::Ptr cloud (new PointCloud());
            PointCloud::Ptr cloud_transformed (new PointCloud());

            if (pcl::io::loadPCDFile<pcl::PointXYZ> (data_path_, *cloud) == -1){
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
            }
            std::cout << "===========================================================\n" << std::endl;
            std::cout << "Loaded\n"
                        << cloud->width * cloud->height
                        << "data points from test_pcd.pcd with the following fields:\n"
                        << std::endl;
            std::cout << "===========================================================\n" << std::endl;

            sensor_msgs::PointCloud2 sensor_cloud;
            pcl::toROSMsg(*cloud, sensor_cloud);

            cloud->header.frame_id = "camera_link";
            sensor_cloud.header.frame_id = "camera_link";
            static tf::TransformBroadcaster br;

            ros::Rate loop_rate(50);
            while (ros::ok()){
                pub_cloud_.publish(cloud);
                pub_cloud_sensor_.publish(sensor_cloud);
                loop_rate.sleep();
            }
            return 0;
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pointcloud_publisher_node");
    PointcloudPublisherNode pp;
    pp.main();
}
