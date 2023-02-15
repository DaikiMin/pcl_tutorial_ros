cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorial_ros)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package( catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package (PCL 1.8 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)  #Specify pcl1.8 path
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_tutorial_ros
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(save_pcd src/others/save_pcd.cpp)
target_link_libraries(save_pcd  ${catkin_LIBRARIES})

add_executable(pointcloud_publisher src/others/pointcloud_publisher.cpp)
target_link_libraries(pointcloud_publisher  ${catkin_LIBRARIES})

add_executable(pointcloud_subscriber src/basic/pointcloud_subscriber.cpp)
target_link_libraries(pointcloud_subscriber  ${catkin_LIBRARIES})

add_executable(passthrough src/basic/passthrough.cpp)
target_link_libraries(passthrough  ${catkin_LIBRARIES})

add_executable(voxel_grid src/basic/voxel_grid.cpp)
target_link_libraries(voxel_grid  ${catkin_LIBRARIES})

add_executable(clustering src/basic/clustering.cpp)
target_link_libraries(clustering  ${catkin_LIBRARIES})

add_executable(plane_detection src/advanced/plane_detection.cpp
                               src/basic/basic_point_cloud_handle.cpp)
target_link_libraries(plane_detection  ${catkin_LIBRARIES})

add_executable(region_growing_segmentation src/advanced/region_growing_segmentation.cpp
                               src/basic/basic_point_cloud_handle.cpp)
target_link_libraries(region_growing_segmentation  ${catkin_LIBRARIES})

add_executable(tracker src/advanced/tracker.cpp
                        src/advanced/point_cloud_tracker.cpp
                        src/basic/basic_point_cloud_handle.cpp)
target_link_libraries(tracker  ${catkin_LIBRARIES} ${PCL_LIBRARIES})