# pointcloud_subscriber

pointcloud_publisherでPublishされたsensor_msgs/PointCloud2型のデータをSubscribeします．
launch内のtarget_frameやtopic_nameを書き換えればXtionやURGのセンサデータをSubscribeすることも可能．  

このプログラムでは，
Subscribeするデータがsensor_msgs/PointCloud2型であるためPCLを扱うことができない．  
そのため，sensor_msgs/PointCloud2データをpcl/PointCloudに変換を行います．  

```cpp
pcl::fromROSMsg<PointT>( *input_cloud, cloud_src ); // sensor_msgs::PointCloud2 -> PointCloud
```

また，基準となる座標フレームを整えるために，tfによる座標変換を行います．  
 
```cpp
tf::TransformListener tf_listener_;
// transform frame :
tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
```

- [sample srcはこちら](../../src/basic/pointcloud_subscriber.cpp)  
- [sample launchはこちら](../../launch/basic/pointcloud_subscriber.launch)  

【 sample launch 】
```py
$ roslaunch pcl_tutorial_ros pointcloud_subscriber.launch
```

launchを起動させると，センサデータの点群の個数が出力される．

[目次に戻る](https://github.com/DaikiMin/pcl_tutorial_ros)