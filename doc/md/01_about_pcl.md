# Point Cloud Library (PCL) とは
ポイントクラウド（point cloud, 点群）を処理するためのライブラリ  
[参考](https://predator.hateblo.jp/entry/2014/10/13/192947)  

**ポイントクラウド**：物体などを点の集合で表現したもの  

- [公式サイト](http://pointclouds.org/)  
- - [Documentation](http://pointclouds.org/documentation/)  
- [Point Cloud Library (PCL)の各モジュールの概要](https://myenigma.hatenablog.com/entry/2016/04/30/152845)  

## PCLのデータ構造
PCLを扱う中で様々なデータ構造が出てきます．その中からいくつかを紹介していきます．

---

### 1. PointCloud
点群データを管理するためのデータ構造であり，様々な点群情報に合わせ，たくさんの型が用意されている．  
もっともシンプルなのがPointXYZ型である．これは，3D xyz情報のみを表すため，最もよく使用されるデータ型の1つです．  
ユーザーは，たとえばx座標にアクセスするために，points[i].data[0] またはpoints[i].xと指定するとアクセスできます．  

```cpp
union
{
  float data[4];
  struct
  {
    float x;
    float y;
    float z;
  };
};
```

【 宣言例 】
```cpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloudpcl::PointXYZRGB);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloudpcl::PointXYZ);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloudpcl::PointXYZI);

/* 以下のように略して記述することも可能 */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud (new PointCloud());
```

他にも3D xyz情報とRGBA情報が含むPointXYZRGBA型や表面の法線と曲率とともにXYZデータを保持するNormalTormal型などがある．  
その他の型については，以下を参照．  
- [Documentation](http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php)  

---

### 2. PointIndices
配列の要素番号が保持されるデータ構造である．  
これは，クラスタリングの結果や平面検出でのインライア・アウトライアの格納するときに使われる．

PointIndicesは，PointCloudの点群情報が格納されれいるデータ(points)の要素番号を記録することにより，xyz情報やRGBA情報などの情報を持たないため，データ量の節約になる利点がある．  

【 宣言例 】
```cpp
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

/* クラスタリングなどの複数のPointIndicesを使うとき */
std::vector<pcl::PointIndices> cluster_indices;
```

※ [インライア・アウトライア](https://mueda-masi.hatenadiary.org/entry/20101002/1286042939)

---

### 3.ModelCoefficients
平面検出など行うときに平面や線などの各モデル合わせたパラメータ(係数)が格納されるデータ構造である．  
例えば，平面の場合のパラメータは4つ（xyz座標空間上の平面の方程式は，ax+by+cz+d=0 ）．  

【 宣言例 】
```cpp
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
```
---

## PCLをROSのパッケージで使うには
### ROSパッケージを作成
PCLを使用したROSパッケージを作成するためには，"pcl_ros" "pcl_conversions"の2つのライブラリが必要になります

例
```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg my_pcl_tutorial_ros pcl_ros pcl_conversions roscpp rospy sensor_msgs std_msgs visualization_msgs
```

### "CMakeLists.txt"と"package.xml"への追記
- **パーティクルフィルタの機能(pcl/tracking/tracking.hなど)を使用するときに"CMakeLists.txt"に以下を追記する必要があります**
```txt
find_package (PCL 1.8 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)  #Specify pcl1.8 path
add_definitions(${PCL_DEFINITIONS})

include_directories( ... ${PCL_INCLUDE_DIRS})

target_link_libraries( ... ${PCL_LIBRARIES})    # 必要な場合がある
```
※catkin_makeで「undefined reference to "pcl::〜"」などが出る場合は，「target_link_libraries( ... ${PCL_LIBRARIES}) 」が必要になります．


また，"package.xml"に以下を追記する必要があります
```txt
 <build_depend>libpcl-all-dev</build_depend>
 <exec_depend>libpcl-all</exec_depend>
```

本パッケージでも，同様の記述が書かれています.

[目次に戻る](https://github.com/DaikiMin/pcl_tutorial_ros)