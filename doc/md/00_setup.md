# setup
## 1. git clone
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/DaikiMin/pcl_tutorial_ros.git
```

## 2. エラーの対処(PCL1.8を使用している場合)
PCL1.8にはsample_consensusにエラーがあります．
そのため，エラーを修正する必要があります．

エラー文
```bash
c:\program files\pcl 1.8.1\include\pcl-1.8\pcl\sample_consensus\model_types.h(99): error C4996: 'pcl::SAC_SAMPLE_SIZE': This map is deprecated and is kept only to prevent breaking existing user code.  Starting from PCL 1.8.0 model sample size is a protected member of the SampleConsensusModel class
```

### 対処方法
model_types.hで該当するコードをコメントアウトすることで対処できます．
```bash
$ cd /usr/include/pcl-1.8/pcl/sample_consensus/
$ sudo vi model_types.h
```

コメントアウトは以下の部分です．
```cpp
namespace pcl 
{
  const static std::map<pcl::SacModel, unsigned int>
 // PCL_DEPRECATED("This map is deprecated and is kept only to prevent breaking "
 //                "existing user code. Starting from PCL 1.8.0 model sample size "
 //                "is a protected member of the SampleConsensusModel class")
  SAC_SAMPLE_SIZE (sample_size_pairs, sample_size_pairs + sizeof (sample_size_pairs) / sizeof (SampleSizeModel));
}
```
[vimの操作方法](https://qiita.com/okamos/items/c97970ab34ff55ff3167)はこちら

## 3. パッケージをビルド
```bash
$ cd ~/catkin_ws/
$ catkin_make
```

※ catkin_makeで以下のようなエラーが出た場合，
```bash
c++: internal compiler error: Killed (program cc1plus)
Please submit a full bug report,
with preprocessed source if appropriate.
```

次のコマンドでcatkin_makeしてください
```bash
catkin_make -j 1
```