#include "pcl_tutorial_ros/point_cloud_tracker.h"

namespace pct {
    void PointCloudTracker::addVector2UnitVector4PlaneAngle ( const geometry_msgs::Point& vec_a, const geometry_msgs::Point& vec_b, 
        const std::string &plane, geometry_msgs::Point* added_vec ) {
        double angle;
        geometry_msgs::Point uni_vector;
        if ( plane == "xy" ) {
            angle = atan2 ( vec_a.z - vec_b.z, vec_a.x - vec_b.x );
            uni_vector.x += cos ( angle );
            uni_vector.y += sin ( angle );
        } else if ( plane == "yz" ) {
            angle = atan2 ( vec_a.y - vec_b.y, vec_a.x - vec_b.x );
            uni_vector.x += cos ( angle );
            uni_vector.y += sin ( angle );
        } else {
            ROS_ERROR( "Please set the plane correctly" );
        }
        added_vec->x += uni_vector.x;
        added_vec->y += uni_vector.y;
        return;
    }
    geometry_msgs::Point PointCloudTracker::clacAverageSpatialVectorOrientation ( geometry_msgs::Point& vec_xy, geometry_msgs::Point& vec_yz, const double length ) {
        geometry_msgs::Point vec_orien;
        vec_xy.x = vec_xy.x / length;
        vec_xy.y = vec_xy.y / length;
        vec_yz.x = vec_yz.x / length;
        vec_yz.y = vec_yz.y / length;
        vec_orien.y = std::atan2 ( vec_xy.y, vec_xy.x ); 
        vec_orien.z = std::atan2 ( vec_yz.y, vec_yz.x ); 
        return vec_orien;
    }
    PointCloudTracker::PointCloudTracker ( ) {
        // ParticleFilterTracker :
        std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;
        std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
        pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> *tracker
        (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> (8));
        //Set the size of one particle 
        ParticleT bin_size;
        bin_size.x = 0.1f;
        bin_size.y = 0.1f;
        bin_size.z = 0.1f;
        bin_size.roll = 0.1f;
        bin_size.pitch = 0.1f;
        bin_size.yaw = 0.1f;
        // Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
        tracker->setMaximumParticleNum(1000);
        tracker->setDelta(0.99);
        tracker->setEpsilon(0.2);
        tracker->setBinSize(bin_size);
        // Set all parameters for  ParticleFilter
        tracker_ = tracker;
        tracker_->setTrans (Eigen::Affine3f::Identity ());
        tracker_->setStepNoiseCovariance (default_step_covariance);
        tracker_->setInitialNoiseCovariance (initial_noise_covariance);
        tracker_->setInitialNoiseMean (default_initial_mean);
        tracker_->setIterationNum (1);
        tracker_->setParticleNum (600);
        tracker_->setResampleLikelihoodThr(0.00);
        tracker_->setUseNormal (false);
        //Setup coherence object for tracking
        pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence 
        (new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>);
        pcl::tracking::DistanceCoherence<PointT>::Ptr distance_coherence 
        (new pcl::tracking::DistanceCoherence<PointT>);
        coherence->addPointCoherence (distance_coherence);
        pcl::search::Octree<PointT>::Ptr search (new pcl::search::Octree<PointT> (0.05));
        coherence->setSearchMethod (search);
        coherence->setMaximumDistance (0.05);
        tracker_->setCloudCoherence (coherence);
        // allocate Memory :
        cloud_tracked_target_.reset ( new PointCloud() );
        // target_locus_ clear : 
        target_locus_.clear();
    }
    // Set Particle Filter :
    bool PointCloudTracker::setTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt ) {
        try {
            Eigen::Affine3f trans = Eigen::Affine3f::Identity();
            trans.translation ().matrix () = Eigen::Vector3f ( ref_pt.x, ref_pt.y, ref_pt.z );
            pcl::transformPointCloud<PointT> ( *ref_cloud, *cloud_tracked_target_, trans.inverse() );
            tracker_->resetTracking();
            tracker_->setReferenceCloud ( cloud_tracked_target_ );
            tracker_->setTrans ( trans );
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // change track target :
    bool PointCloudTracker::changeTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt ) {
        try {
            Eigen::Affine3f trans = Eigen::Affine3f::Identity();
            trans.translation ().matrix () = Eigen::Vector3f ( ref_pt.x, ref_pt.y, ref_pt.z );
            pcl::transformPointCloud<PointT> ( *ref_cloud, *cloud_tracked_target_, trans.inverse() );
            tracker_->setReferenceCloud ( cloud_tracked_target_ );
            tracker_->setTrans ( trans );
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Get Particle Filter Tracking Result :
    bool PointCloudTracker::getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Pose* output_pose ) {
        try {
            tracker_->setInputCloud ( input_cloud );
            tracker_->compute ( );
            ParticleT pf_result = tracker_->getResult ();
            if (!std::isfinite (pf_result.x) || 
                !std::isfinite (pf_result.y) || 
                !std::isfinite (pf_result.z)) {
                ROS_ERROR( "Tracking failed : TrackResult is Nan data." );
                output_pose->position.x = 0.0;
                output_pose->position.y = 0.0;
                output_pose->position.z = 0.0;
                output_pose->orientation.x = 0.0;
                output_pose->orientation.y = 0.0;
                output_pose->orientation.z = 0.0;
                return false;
            } else {
                // Get Target PointCloud :
                Eigen::Affine3f transformation = tracker_->toEigenMatrix ( pf_result );
                pcl::transformPointCloud<PointT> ( *(tracker_->getReferenceCloud ()), *output_cloud, transformation );
                // Get Target Pose :
                output_pose->position.x = pf_result.x;
                output_pose->position.y = pf_result.y;
                output_pose->position.z = pf_result.z;
                tf::Quaternion quat = tf::createQuaternionFromRPY( pf_result.roll, pf_result.pitch, pf_result.yaw );
                quaternionTFToMsg( quat, output_pose->orientation ); 
                // add locus :
                target_locus_.push_back( output_pose->position );
                if ( target_locus_.size () > 10 ) target_locus_.erase ( target_locus_.begin( ) );
                return true;
            }
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Get Particle Filter Tracking Result :
    bool PointCloudTracker::getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Point* output_pt ) {
        try {
            tracker_->setInputCloud ( input_cloud );
            tracker_->compute ( );
            ParticleT pf_result = tracker_->getResult ();
            if (!std::isfinite (pf_result.x) || 
                !std::isfinite (pf_result.y) || 
                !std::isfinite (pf_result.z)) {
                ROS_ERROR( "Tracking failed : TrackResult is Nan data." );
                output_pt->x = 0.0;
                output_pt->y = 0.0;
                output_pt->z = 0.0;
                return false;
            } else {
                // Get Target PointCloud :
                Eigen::Affine3f transformation = tracker_->toEigenMatrix ( pf_result );
                pcl::transformPointCloud<PointT> ( *(tracker_->getReferenceCloud ()), *output_cloud, transformation );
                // Get Target Pose :
                output_pt->x = pf_result.x;
                output_pt->y = pf_result.y;
                output_pt->z = pf_result.z; 
                // add locus :
                target_locus_.push_back( *output_pt );
                if ( target_locus_.size () > 10 ) target_locus_.erase ( target_locus_.begin( ) );
                return true;
            }
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Get the current particles :
    bool PointCloudTracker::getParticles ( PointCloud::Ptr output_cloud ) {
        try {
            ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
            PointCloud::Ptr tmp ( new PointCloud() );   
            //Set pointCloud with particle's points
            for( auto &particle : particles->points ) {
                PointT point;
                point.x = particle.x;
                point.y = particle.y;
                point.z = particle.z;
                tmp->points.push_back (point);
            }
            *output_cloud = *tmp;
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Get the current target motion(3D ver) :
    bool PointCloudTracker::getTargetMotion3D ( double* distance, geometry_msgs::Point* orientation ) {
        try {
            double length = 0.0;
            double ave_vec_mag = 0.0;
            geometry_msgs::Point some_vec_xy;
            geometry_msgs::Point some_vec_yz;
            geometry_msgs::Point pre_val;
            for ( auto& locus : target_locus_ ) {
                if ( length == 0.0 ) {
                    pre_val = locus;
                } else {
                    ave_vec_mag += calcSpatialVectorMagnitude ( locus, pre_val );
                    addVector2UnitVector4PlaneAngle ( locus, pre_val, "xy", &some_vec_xy );
                    addVector2UnitVector4PlaneAngle ( locus, pre_val, "yz", &some_vec_yz );
                    pre_val = locus;
                }
                length ++;
            }
            *distance = ave_vec_mag / length;
            *orientation = clacAverageSpatialVectorOrientation ( some_vec_xy, some_vec_yz, length );
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
    // Get the current target motion(2D ver) :
    bool PointCloudTracker::getTargetMotion2D ( double* distance, double* orientation ) {
        try {
            double length = 0.0;
            double ave_vec_mag = 0.0;
            geometry_msgs::Point vector_ave;
            geometry_msgs::Point pre_val;
            for ( auto& locus : target_locus_ ) {
                if ( length == 0.0 ) {
                    pre_val = locus;
                } else {
                    ave_vec_mag += hypotf ( locus.x - pre_val.x, locus.y - pre_val.y );  
                    double angle = atan2 ( locus.y - pre_val.y, locus.x - pre_val.x );                 
                    vector_ave.x += cos ( angle );
                    vector_ave.y += sin ( angle );
                    pre_val = locus;
                }
                length ++;
            }
            vector_ave.x = vector_ave.x / length;
            vector_ave.y = vector_ave.y / length;
            *distance = ave_vec_mag / length;
            *orientation = atan2 ( vector_ave.y, vector_ave.x );
            return true;
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
}