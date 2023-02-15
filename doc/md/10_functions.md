# 便利な関数たち
ここからは，PCLで役に立つ関数たちを紹介

1. [pcl_common](#1pcl_common)
    - pcl::getMinMax3D
    - pcl::compute3DCentroid
2. [NearestNeighborSearch](#2nearest-neighbor-search)
    - K nearest neighbor search
    - Neighbors within radius search
3. [pcl::fromROSMsg/pcl::toROSMsg](#3pclfromrosmsg-pcltorosmsg)
    - pcl::fromROSMsg
    - pcl::toROSMsg
4. [transformLaserScanToPointCloud](#4transformlaserscantopointcloud)

---

## 1.pcl_common
PCLのライブラリの大半で使用される一般的なデータ構造とメソッドたち  

- [Documentationはこちら](https://pointclouds.org/documentation/group__common.html)  


pcl_commonの中からいくつか紹介  
### pcl::getMinMax3D
指定された点群の3つの（xyz）次元それぞれの最小値と最大値を取得します．

```cpp
void pcl::getMinMax3D ( const pcl::PointCloud< PointT > &    cloud,
                        const std::vector< int > &          indices,
                        Eigen::Vector4f &                   min_pt,
                        Eigen::Vector4f &                   max_pt
                      )
```

[ Parameters ]  
cloud：     点群データメッセージ  
indices：   クラウドから使用するポイントインデックスのベクトル  
min_pt：    結果の最小値  
max_pt：    結果の最大値  

- [Documentationはこちら](https://pointclouds.org/documentation/group__common.html#ga3166f09aafd659f69dc75e63f5e10f81)  
※ 引数の取り方には複数あるので上を参考にすること  

---

### pcl::compute3DCentroid
PointCloudの重心を計算します．

```cpp
unsigned int pcl::compute3DCentroid ( const pcl::PointCloud< PointT > &  cloud,
                                        Eigen::Matrix< Scalar, 4, 1 > &  centroid
                                    )
```

[ Parameters ]  
cloud：     点群データメッセージ  
centroid：  結果の重心  

- [Documentationはこちら](https://pointclouds.org/documentation/group__common.html#gaf5729fae15603888b49743b118025290)  
※ 引数の取り方には複数あるので上を参考にすること  

---

## 2.Nearest neighbor search
最近傍探索をする関数たち  
- [Documentationはこちら](https://pcl.readthedocs.io/projects/tutorials/en/master/kdtree_search.html#kdtree-search)  

**K nearest neighbor search**  
指定されたポイントのk個の最近傍点を検索します．

```cpp
virtual int pcl::KdTree< PointT >::nearestKSearch (
        const PointT &          p_q,
        int                     k,
        std::vector< int > &    k_indices,
        std::vector< float > &  k_sqr_distances
    )
```

[ Parameters ]  
p_q：               指定されたポイント  
k：                 検索する最近傍点の数  
k_indices：         隣接するポイントの結果のインデックス（事前にkにサイズ変更する必要があります！）  
k_sqr_distances：   隣接するポイントまでの結果の平方距離（事前にkにサイズ変更する必要があります！）  

[ Return ]  
見つかった最近傍点の数 

---

**Neighbors within radius search**  
指定されたポイントから指定された半径内にあるすべての最近傍点を検索します．

```cpp
virtual int pcl::KdTree< PointT >::radiusSearch (
        const PointT &          p_q,
        double                  radius,
        std::vector< int > &    k_indices,
        std::vector< float > &  k_sqr_distances,
        unsigned int            max_nn = 0
        )
```

[ Parameters ]  
p_q：               指定されたポイント  
radius：            p_qのすべての近傍を囲む球体の半径  
k_indices：         隣接する点の結果のインデックス  
k_sqr_distances：   結果として得られる隣接点までの距離の平方  
max_nn：            与えられた場合，返される隣接の最大値をこの値に制限します．max_nnが0または入力クラウド内のポイントの数よりも大きい数に設定されている場合，半径内のすべての近隣が返されます．  

[ Return ]  
radiusで見つかった最近傍点の数  

---

## 3.pcl::fromROSMsg /  pcl::toROSMsg
### pcl::fromROSMsg
sensor_msgs :: PointCloud2 を pcl :: PointCloud < T >に変換する関数  

```cpp
void pcl :: fromROSMsg（
        const sensor_msgs :: PointCloud2＆  cloud，
        pcl :: PointCloud < T >＆           pcl_cloud
    )
```
  
- [Documentationはこちら](http://docs.ros.org/indigo/api/pcl_conversions/html/namespacepcl.html#af662c7d46db4cf6f7cfdc2aaf4439760)  

### pcl::toROSMsg
pcl :: PointCloud < T > を sensor_msgs :: PointCloud2に変換する関数  

```cpp
void pcl::moveFromROSMsg(
        sensor_msgs::PointCloud2 &  cloud,
        pcl::PointCloud< T > &      pcl_cloud
    )
```

- [Documentationはこちら](http://docs.ros.org/indigo/api/pcl_conversions/html/namespacepcl.html#abb8b3a2632e07dae0b541a257898c8a8)  

---

## 4.transformLaserScanToPointCloud
URGなどのLaserScan型をPointCloud2型に変換する関数．これによりLaserScan型のデータもPCLで扱うことができる．  

```cpp 
void laser_geometry::LaserProjection::transformLaserScanToPointCloud(
        const std::string &             target_frame,
        const sensor_msgs::LaserScan &  scan_in,
        sensor_msgs::PointCloud &       cloud_out,
        tf::Transformer &               tf
)
```

[ Parameters ]  
target_frame：  結果の点群のフレーム  
scan_in：       入力レーザースキャン  
cloud_out：     出力点群  
tf：            変換の実行に使用するtf :: Transformerオブジェクト  

- [Documentationはこちら](https://docs.ros.org/diamondback/api/laser_geometry/html/classlaser__geometry_1_1LaserProjection.html#ab7ce0e0fa1bb6ab0f6d7d477d71284e9)  

---

[目次に戻る](https://github.com/DaikiMin/pcl_tutorial_ros)