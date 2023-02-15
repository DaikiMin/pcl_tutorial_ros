#ifndef POINT_CLOUD_TRACKER
#define POINT_CLOUD_TRACKER

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<PointT, ParticleT> ParticleFilter;

namespace pct {

    class PointCloudTracker {
        private :
            ParticleFilter *tracker_;
            PointCloud::Ptr cloud_tracked_target_;
            std::vector<geometry_msgs::Point> target_locus_;
            double calcSpatialVectorMagnitude ( const geometry_msgs::Point& vec_a, const geometry_msgs::Point& vec_b );
            void addVector2UnitVector4PlaneAngle ( const geometry_msgs::Point& vec_a, const geometry_msgs::Point& vec_b, const std::string &plane, geometry_msgs::Point* added_vec );
            geometry_msgs::Point clacAverageSpatialVectorOrientation ( geometry_msgs::Point& vec_xy, geometry_msgs::Point& vec_yz, const double length );
        public :
            PointCloudTracker ( );
            // Set Particle Filter :
            bool setTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt );
            // change track target :
            bool changeTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt );
            // Get Particle Filter Tracking Result :
            bool getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Pose* output_pose );
            // Get Particle Filter Tracking Result :
            bool getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Point* output_pt );
            // Get the current particles :
            bool getParticles ( PointCloud::Ptr output_cloud );
            // Get the current target motion(3D ver) :
            bool getTargetMotion3D ( double* distance, geometry_msgs::Point* orientation );
            // Get the current target motion(2D ver) :
            bool getTargetMotion2D ( double* distance, double* orientation );
    };

    inline double PointCloudTracker::calcSpatialVectorMagnitude ( const geometry_msgs::Point& vec_a, const geometry_msgs::Point& vec_b ) {
        return std::sqrt( std::pow( vec_a.x - vec_b.x, 2.0 ) + std::pow( vec_a.y - vec_b.y, 2.0 ) + std::pow( vec_a.z - vec_b.z, 2.0 ) );
    }

}

#endif