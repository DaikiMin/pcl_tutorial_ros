# pcl_tutorial_ros
Point Cloud Libraryのチュートリアル

# 前提知識
0. ROSに関する知識(topicやtfなど)
1. C++に関する知識(関数やオブジェクト指向など)

## **目次**

0. [**setup**](doc/md/00_setup.md)

1. [**Point Cloud Library (PCL) とは**](doc/md/01_about_pcl.md)
    - PCLのデータ構造
        1. PointCloud
        2. PointIndices
        3. ModelCoefficients
    - PCLをROSのパッケージで使うには

---

2. [**Before Tutorial**](doc/md/02_before_tutorial.md)
    1. pointcloud_publisher

---

3. **Tutorial**
    - **Basic**
        1. [pointcloud_subscriber](doc/md/03_pointcloud_subscriber.md)
        2. [PassThrough](doc/md/04_passthrough.md)
        3. [VoxelGrid](doc/md/05_voxelgrid.md)
        4. [Clustering & Basic編まとめ](doc/md/06_clustering.md)

    - **Advanced**
        1. [PlaneDetection](doc/md/07_plane_detection.md)
        2. [RegionGrowingSegmentation](doc/md/08_region_growing_segmentation.md)  
        3. [Tracker](doc/md/09_tracker.md)  

---

4. [**便利な関数たち**](doc/md/10_functions.md)
    1. pcl_common
        - pcl::getMinMax3D
        - pcl::compute3DCentroid
    2. NearestNeighborSearch
        - K nearest neighbor search
        - Neighbors within radius search
    3. pcl::fromROSMsg/pcl::toROSMsg
        - pcl::fromROSMsg
        - pcl::toROSMsg
    4. transformLaserScanToPointCloud

---
5. [**終わりに & Reference**](doc/md/11_conclusion.md)

---
