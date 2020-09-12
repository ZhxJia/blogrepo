---
title: apollo 融合算法--fusion component
tags:
- sensor fusion
- apollo
categories:
- autonomous
- apollo
mathjax: true
---

多传感器融合将各种传感器进行多层次，多空间的信息互补和优化组合处理，最终产生对观测环境的一致性解释。在自动驾驶平台中稳定可靠的感知算法仍然是有待解决的问题，多个传感器信息融合无疑成为提升自动驾驶安全性的趋势。

<img src="apollo-fusion\sensorfusion.png" alt="sensorfusion" style="zoom: 33%;" />

<!--more-->

## 一、组件初始化和相关配置

### 1. 外部配置文件

```protobuf
  components {
    class_name: "FusionComponent"
    config {
      name: "SensorFusion"
      config_file_path: "/apollo/modules/perception/production/conf/perception/fusion/fusion_component_conf.pb.txt"
      readers {
          channel: "/perception/inner/PrefusedObjects"
        }
    }
  }
```

组件的配置文件：

```protobuf
#../productrion/conf/preception/fusion/fusion_component_conf.pb.txt
fusion_method: "ProbabilisticFusion"
fusion_main_sensor: "velodyne128"
object_in_roi_check: true
radius_for_roi_object_check: 120
output_obstacles_channel_name: "/apollo/perception/obstacles"
output_viz_fused_content_channel_name: "/perception/inner/visualization/FusedObjects"
```

接收来自`fusion_camera_detection_component` ,`recognition_component`,`radar_detection_component`三个相关组件输出的`/percepiton/inner/PrefusedObjects`通道信息，信息格式位于`inner_component_messages.h`的`<SensorFrameMessage>`

该组件的输出通道为`"/apollo/perception/obstacles"` 信息格式为`<PerceptionObstacles>`位于
`modules/perception/proto/preception_obstacle.proto`

```c++
message PerceptionObstacles {
  repeated PerceptionObstacle perception_obstacle = 1;  // An array of obstacles
  optional common.Header header = 2;                    // Header
  optional common.ErrorCode error_code = 3 [default = OK];
  optional LaneMarkers lane_marker = 4;
  optional CIPVInfo cipv_info = 5;  // Closest In Path Vehicle (CIPV)
}
```



### 2. 功能初始化

功能类`<fusion::ObstacleMultiSensorFusion>`初始化:
根据配置文件中的`fusion_method`创建接口类`BaseFusionSystem`的实例化对象指针并初始化`fusion_->Init()`

```c++
  bool ProbabilisticFusion::Init(const FusionInitOptions& init_options) override;
```

由`ConfigManager`管理的该功能类的相关参数为：

|          参数名称           |    默认值     | 参数说明 | 参数类型 |
| :-------------------------: | :-----------: | :------: | :------: |
|          use_lidar          |     true      |          |          |
|          use_radar          |     true      |          |          |
|         use_camera          |     true      |          |          |
|       tracked_method        |  PbfTracker   |          |          |
|   data_association_method   | HMAssociation |          |          |
|     gate_keeper_method      | PbfGatekeeper |          |          |
|     prohibition_sensors     |  radar_front  |          |          |
| max_lidar_invisible_period  |     0.25      |          |          |
| max_radar_invisible_period  |     0.50      |          |          |
| max_camera_invisible_period |     0.75      |          |          |
|    max_cached_frame_num     |      50       |          |          |



**创建初始化`scenes_`,`matcher_`,`gate_keeper_`分别对应`<Scene>`,`<HMtrakersObjectsAssociation>`,`<PbfGatekeeper>`**

- `matcher_->Init()`

  |               参数名称               | 默认值 | 参数说明 | 参数类型 |
  | :----------------------------------: | :----: | :------: | :------: |
  |       s_match_distance_thresh_       |  4.0   |          |          |
  |       s_match_distance_bound_        | 100.0  |          |          |
  | s_association_center_dist_threshold_ |  30.0  | 单位：米 |          |

  > `s_association_center_dist_threshold_` 参数说明：
  > 对于相机与(lidar/radar)的关联，考虑到2d-to-3d的平均误差为7%,30m是200m的15%,是两倍的平均误差。

  

  ```c++
  bool HMTrackersObjectsAssociation::Init() override {
      track_object_distance_.set_distance_thresh(
          static_cast<float>(s_match_distance_thresh_));
      return true;
    }
  ```

- `gate_keeper_->Init()`

  ConfigManager管理的参数：

  |           参数名称           | 默认值 | 参数说明 | 参数类型 |
  | :--------------------------: | :----: | :------: | :------: |
  |     publish_if_has_lidar     |  true  |          |   bool   |
  |     publish_if_has_radar     |  true  |          |   bool   |
  |    publish_if_has_camera     |  true  |          |   bool   |
  |        use_camera_3d         |  true  |          |   bool   |
  | min_radar_confident_distance |   40   |          |          |
  |  max_radar_confident_angle   |   20   |          |          |
  | min_camera_publish_distance  |   50   |          |          |
  |  invisible_period_threshold  | 0.001  |          |          |
  |        toic_threshold        |  0.8   |          |          |
  | use_track_time_pub_strategy  |  true  |          |          |
  |    pub_track_time_thresh     |   3    |          |          |
  |     existance_threshold      |  0.7   |          |          |
  |  radar_existance_threshold   |  0.9   |          |          |

**最后进行数据融合的初始化，包括了三个方面：**

> - TypeFusion
> - ExistanceFusion
> - PbfTracker

(1) `TypeFusion::Init()`

ConfigManager管理的参数，对应各lidar和camera的检测结果的可信度。

```protobuf
#注这里是.pt文件 实际通过GetProtoFromFile加载
camera_params {
  name: "front_6mm"
  valid_dist: 110
  reliability: 0.95
  reliability_for_unknown: 0.2
}

camera_params {
  name: "front_12mm"
  valid_dist: 150
  reliability: 0.5
  reliability_for_unknown: 0.2
}

lidar_params {
  name: "velodyne64"
  reliability: 0.5
  reliability_for_unknown: 0.5
}
```

对应修改`<DstTypeFusionOpitons>`配置参数,默认如下：

```c++
struct DstTypeFusionOptions {
  std::map<std::string, double> camera_max_valid_dist_ = {
      {"camera_smartereye", 110},
      {"camera_front_obstacle", 110},
      {"front_6mm", 110},
      {"camera_front_narrow", 150},
  };
  std::map<std::string, double> sensor_reliability_ = {
      {"velodyne64", 0.5},          {"velodyne_64", 0.5},
      {"velodyne128", 0.5},         {"camera_smartereye", 0.95},
      {"front_6mm", 0.95},          {"camera_front_obstacle", 0.95},
      {"camera_front_narrow", 0.5},
  };
  std::map<std::string, double> sensor_reliability_for_unknown_ = {
      {"velodyne64", 0.5},          {"velodyne_64", 0.5},
      {"velodyne128", 0.5},         {"camera_smartereye", 0.2},
      {"front_6mm", 0.2},           {"camera_front_obstacle", 0.2},
      {"camera_front_narrow", 0.2},
  };
};
```

在TypeFusion初始化的最后一步通过一个单例类`<DstManager>`管理各个子节点和它们之间的相互关系

```c++
// brief: app initialization
// param [in]: app_name (now it is TypeFusion)
// param [in]: fod_subsets, hypotheses sets
// param [in]: fod_subset_names
bool DstManager::AddApp(const std::string &app_name,
                        const std::vector<uint64_t> &fod_subsets,
                        const std::vector<std::string> &fod_subset_names) {...}
```

> 其中fod_subsets为`<DstTypeFusion::DstMaps>`中的：
>
> ```c++
>   std::vector<uint64_t> fod_subsets_ = {
>       PEDESTRIAN=1,     BICYCLE=2, VEHICLE=4, OTHERS_MOVABLE=8,
>       OTHERS_UNMOVABLE=16, OTHERS=24,  UNKNOWN=31};
> ```
>
> 对应的fod_subset_names为：
>
> ```c++
>   std::vector<std::string> subset_names_ = {
>       "PEDESTRIAN",       "BICYCLE", "VEHICLE", "OTHERS_MOVABLE",
>       "OTHERS_UNMOVABLE", "OTHERS",  "UNKNOWN"};
> ```
>
> 用二进制每一位表示一个类型，若是多个类型的复合则将对应位置为1

`<DstManager>`类主要管理的数据结构为：`<DstCommonData>` 将其与各个`app_name`作为一个map

```c++
struct DstCommonData {
  // ensure initialize DSTEvidence once
  bool init_ = false;
  // fods
  size_t fod_loc_ = 0; //the indice of fod
  std::vector<uint64_t> fod_subsets_;
  // for transforming to probability effectively
  std::vector<size_t> fod_subset_cardinalities_; //对应fod_subset中各个值对应二进制1的个数
  std::vector<std::string> fod_subset_names_;
  // for combining two bbas effectively. 对应inter_relation的subsets两两之间索引值
  std::vector<std::vector<std::pair<size_t, size_t>>> combination_relations_;//
  // for computing support vector effectively
  std::vector<std::vector<size_t>> subset_relations_; //对应subsets含有包含关系的subset索引
  // for computing plausibility vector effectively
  std::vector<std::vector<size_t>> inter_relations_; //对应subsets含有相同值的各subset索引
  std::map<uint64_t, size_t> subsets_ind_map_; // pair(subset,indice )
};
```

上述各值通过`<DstManager::AddMap>`进行初值的填充，此处不展开。
inter_relations:   ![image-20200429223959318](apollo-fusion\image-20200429223959318.png)subset_relations:![](apollo-fusion\image-20200429224301475.png)

(2) `DstExistanceFusion::Init() `

```protobuf
track_object_max_match_distance: 4.0

camera_valid_dist {
  camera_name: "camera_front_obstacle"
  valid_dist: 110
}
camera_valid_dist {
  camera_name: "camera_front_narrow"
  valid_dist: 150
}
camera_valid_dist {
  camera_name: "front_6mm"
  valid_dist: 110
}
camera_valid_dist {
  camera_name: "front_12mm"
  valid_dist: 150
}
```

同样最后根据以下maps添加APP

```c++
struct ToicDstMaps {
  // for (N)TOIC: (not)target of interest in camera judgement
  enum { TOIC = (1 << 0), NTOIC = (1 << 1), TOICUNKOWN = (TOIC | NTOIC) };
  std::vector<uint64_t> fod_subsets_ = {TOIC, NTOIC, TOICUNKOWN};
  std::vector<std::string> subset_names_ = {"TOIC", "NTOIC", "TOICUNKOWN"};
};

struct ExistanceDstMaps {
  enum { EXIST = (1 << 0), NEXIST = (1 << 1), EXISTUNKOWN = (EXIST | NEXIST) };
  std::vector<uint64_t> fod_subsets_ = {EXIST, NEXIST, EXISTUNKOWN};
  std::vector<std::string> subset_names_ = {"EXIST", "NEXIST", "EXISTUNKOWN"};
};
```

此处通过` DstManager::Instance()->AddApp`添加两个名称为`DstExistanceFusion` 和`DstToicFusion`的App

> **toic** means :target of interest in camera judgement

(3) `PbfTracker::InitParams()`

|        参数名称         |       默认值       |    参数说明    | 参数类型 |
| :---------------------: | :----------------: | :------------: | :------: |
|   type_fusion_method    |   DstTypeFusion    | 融合方法类名称 |  string  |
|  motion_fusion_method   | KalmanMotionFusion |                |  string  |
|   shape_fusion_method   |   PbfShapeFusion   |                |  string  |
| existance_fusion_method | DstExistanceFusion |                |  string  |



## 二、算法处理

多个传感器发布的`<SensorFrameMessage>`传递到Proc处理函数中，调用内部处理函数：

```c++
// @brief: multi sensor fusion entry function
// @param[in]: in_menssage (the received message)
// @param[out]: out_message (the ouyput message)
// @param[out]: viz_message (for visualizing)

bool FusionComponent::InternalProc(
    const std::shared_ptr<SensorFrameMessage const>& in_message,
    std::shared_ptr<PerceptionObstacles> out_message,
    std::shared_ptr<SensorFrameMessage> viz_message) {...}
```

### 2.1 消息序列化

`Msgserializer::SerializeMsg`

### 2.2融合处理Process( )

`fusion_->Process(frame, &fused_objects)`

```c++
 // @brief: Process MultiSensorFusion
 // @param[in]: frame (SensorFrameMessage->frame_)
 // @param[out]: objects (To obtain the fused objects)
 bool ObstacleMultiSensorFusion::Process(const base::FrameConstPtr& frame,
               std::vector<base::ObjectPtr>* objects);
```

`fusion_->Fuse(options, frame, objects)`

```c++
// @brief: func inteface
// @param[in]: options (FusionOptions ,default null) 
// @param[in]: sensor_frame (SensorFrameMessage->frame_)
// @param[out]: objects (To obtain the fused objects)
bool ProbabilisticFusion::Fuse(const FusionOptions& options,
            const base::FrameConstPtr& sensor_frame,
            std::vector<base::ObjectPtr>* fused_objects) override;
```

1. **Save frame data**
   这里用到了单例类`<SensorDataManager>`用于管理各个传感器帧数据，首先判断传感器是否是`publish_sensor`，即是否是主传感器（用于最用发布融合信息的基准,apollo默认是velodyne128）,从第一次接收主传感器数据开始，添加各传感器测量信息：

   ```c++
   // @brief: add sensor frame message to <Sensor> datastructure
   // @param[in]: frame_ptr (sensor frame)
   
   void SensorDataManager::AddSensorMeasurements(
       const base::FrameConstPtr& frame_ptr) {...}
   ```

   通过调用`Sensor::AddFrame(const base::FrameConstPtr& frame_ptr)`将传感器数据存储到`<SensorFrame>`数据结构中
   这里主要分两步：

   - 找到传感器(对应`<Sensor>`数据结构)，如果没有就创建
   - 然后向对应传感器中添加数据帧frame，存储到`<Sensor>`的`frames_`中(对应的数据存储结构为`<SensorFrame>`)
     这里暗含将原`<frame>`数据结构转换为`<SensorFrame>`数据结构

2. **query related sensor_frames for fusion**
   数据融合的前期数据准备阶段：

   ```c++
   // @brief: get latest frame of each sensor
   // @param[in]: timestamp(the timestamp of frame)
   // @param[out]: frames (the realted frames)
   void SensorDataManager::GetLatestFrames(
       double timestamp, std::vector<SensorFramePtr>* frames) const {...}
   ```

   以timestamp为依据，遍历当前的所有传感器`<Sensor>`,查询数据帧`<SensorFrame>` 范围
   (_lastest_fused_time_stamp,time_stamp) 即上一次查询(融合)的时间到当前查询的时间范围内最新帧，并按照时间顺序由早到晚排序。

3. **preform fusion on realted frames**
   对于related frames中的每一帧，执行：

   ```c++
   // @brief: perform fusion
   // @param[in]: frame (each frame of the realted frames)
   void ProbabilisticFusion::FuseFrame(const SensorFramePtr& frame) {...}
   ```

   融合过程有三个步骤，每一个步骤又分为关联，更新，创建新的Track，注意下方函数均是对单一数据帧中的objects和tracks而言：

   - ProbabilisticFusion::FusedForegroundTrack 前景融合

     - HMTrackersObjectsAssociation::Associate
       (`sensor_objects <----> fusion_tracks`) 得到 `association_result`

       - HMTrackersObjectsAssociation::IdAssign() 
         得到assignments,unassigned_tracks,unassigned_measurements ；
          sensor_objects与相同传感器的tracks进行匹配，匹配依据是base_object->track_id

       - HMTrackersObjectsAssociation::ComputeAssociationDistanceMat()
         得到association_mat 关联距离矩阵(unassigned_tracks,unassigned_measurements)

         - TrackObjectDistance::Compute()
           计算fused track 和 sensor object之间的距离，结合阈值判断当前检测sensor_object与各传感器对应的fused track中的object的距离(polygon distance)度量，然后其中的最小值作为最终的度量添加到关联矩阵中。
           **1** 当新检测的sensor_object是**Lidar**时，计算与fused_track各个传感器的最新object的距离度量：

           - TrackObjectDistance::ComputeLidarLidar(）

             sensor_object(lidar检测)与fused_object(lidar检测)之间距离小于10m，计算Polygon中心之间的欧氏距离，否则返回`std::numeric_limits<float>::max`，即不进行匹配

           - TrackObjectDistance::ComputeLidarRadar(）

             sensor_object(lidar检测)与fused_object(radar检测)之间距离小于10m,计算Polygon中心之间的欧式距离，否则返回`std::numeric_limits<float>::max` 即不进行匹配

           - TrackObjectDistance::ComputeLidarCamera(）
             lidar与camera的匹配距离计算则稍显复杂，首先先判断是否lidar自身传感器的跟踪连续，若不连续则判断sensor_object(lidar检测)与fused_object(camera检测)是否大于**动态阈值**，若是则直接返回`distance_thresh_=4.0`,否则，计算sensor_object(lidar)点云在相机图像平面的投影object与fused_object(camera检测)的相似性：

             - ComputePtsBoxSimilarity()

               计算点云的2d投影box和相机的2d投影box之间的相似性,取值范围[0,1]，方式是通过计算位置的相似性和形状的相似性，然后进行将这两个相似性融合。

               - ComputePtsBoxLocationSimilarity()

                 计算点云和相机box的位置相似性，首先计算IOU(此处将box扩大一定尺寸再计算
                 IOU)，若没有交集则返回最小相似性1e-6，然后按照如下步骤：

                 1. 计算lidar投影的2d点位于camera box之外的各个点与camera box边界的平均距离
                 2. 根据camera box的大小归一化这个距离，然后假设归一化后的距离服从高斯分布
                 3. 假设归一化后的x,y方向像素差值服从正态分布，则它们转化为标准正太分布后的平方和服从卡方分布，通过卡方分布，得到相似性：
                    ![img](apollo-fusion\chi_squared.png)

               - ComputePtsBoxShapeSimilarity()
                 计算点云和相机box的形状的相似性[0,1]:

                 1. 首先计算box尺寸的差异，针对点云只检测到一个点的，此处限制sensor_object(点云检测)的尺寸不能低于camera box的十分之一
                 2. 将随机变量标准化(根据先验的标准差)，使其服从标准正态分布
                 3. 同样根据卡方分布得到相似度

               - FuseTwoProbabilities()

                 将上述两个概率融合(即相似度),当两概率之和大于1，融合概率大于0.5，否则小于0.5，得到的相似性[0,1]乘以距离阈值`distance_thresh_=4.0`即可得到最终的距离度量
                 $$
                 tmp=\frac{p1}{1-p1}*\frac{p2}{1-p2}\\
                 prob = \frac{tmp}{tmp+1}
                 $$

             - QueryProjectedVeloCtOnCamera()

               如果lidar检测目标的cloud为空(**此处疑问：为啥有检测目标还会为空**)，则将lidar检测的物体中心投影到相机图像坐标系中，计算中心点之间的距离(单位为像素)。

             **综上，通过计算lidar camera物体的相似性或者中心的距离得到lidar检测目标和camera目标之间的距离度量(此处最大值由distance_thresh_=4.0限定范围)**

           **2** 当新检测到的sensor_object是radar时，基本与lidar时同样的操作，计算lidar 与radar之间的距离度量(与上面的相似)和camera与radar之间的距离度量：

           - TrackObjectDistance::ComputeRadarCamera(）
             与lidar和camera的距离度量相似，首先计算camera可用的信息：包括camera检测的2d box，物体的中心点以及box的宽和高；radar可用的信息：radar的检测物体的3d中心点以及8个顶点，将8个顶点的高度修改为相机检测物体的高度，然后都投影到2d图像平面。
             紧接着计算相似性，若radar检测的物体在相机的视场内，相似性度量方式如下：
             - ComputeRadarCameraXSimilarity()
               计算中心点在x方向上的差异，并进行通过camera box size_x进行归一化
               通过`WelshVarLossFun`计算相似性，然后限制最大概率(相似度)为(0.9)(进行缩放)，下图为
               welshvarlossfun示意图，以0.5为阈值分为两段,横轴为距离，纵轴为概率。
               <img src="apollo-fusion\image-20200508214807548.png" alt="image-20200508214807548" style="zoom:30%;" />
             - ComputeRadarCameraYSimilarity()
               计算2dbox中心点在y方向上的差异(单位为像素)，然后根据给定的标准差进行normalize,同样通过卡方分布获取相似性，最后根据给定的范围缩放概率到区间(0.5,0.6)
             - ComputeRadarCameraLocSimilarity()
               计算物体3D center的位置差异(单位为米)，通过`WelshVarLossFun`获取相似性，然后进行缩放概率区间为(0,0.7)
               <img src="apollo-fusion\image-20200508222547038.png" alt="image-20200508222547038" style="zoom:30%;" />
             - ComputeRadarCameraVelocitySimilarity()
               当camera或者radar的速度大于2.0时，计算速度差值，并通过卡方分布得到相似性分数，然后限制缩放(>0.5，即positive)的概率为最大0.9
             - FuseMultipleProbabilities()
               最后将这多个概率融合，作为最终的相似度距离度量，融合过程概率先取对数，将乘法变为加法，然后再取指数。
           - TrackObjectDistance::BuildProjectionCacheObject()
             由于一般Lidar是360度视角，而相机视角有限，在进行距离计算时，需要判断lidar的点是否位于相机的视锥体(Frustum)中，此处采用的方式是将lidar点云投影到相机坐标系的图像平面中2d点中并判断是否超出了图像边界，将在相机视场内时，将lidar点的图像平面投影点保存到数据结构
             `<ProjectCache>`中，该数据结构存储各数据帧各个物体lidar 2d图像投影点(注：前提是这些点的原3d点位于相机视场内)`<ProjectionCacheObject>` 为各个物体的基本存储单位，保存该物体的投影点的索引和2d边界框。

           **3** 当新检测到的sensor_object是camera时，只计算与跟踪列表中lidar的检测目标的相似性，关于相机和相机本身的已经在相机检测组件中进行了融合。

       - HMTrackersObjectsAssociation::MinimizeAssignment()
         根据上一步得到的距离矩阵通过匈牙利算法`common::GatedHungarianMatcher<float> optimizer_`进行二分图最优匹配，距离阈值4.0(用于判断连接是否有效),距离bound为100，得到`assignments`,`unassigned`
         `_tracks,unassigned_measurements`

       - HMTrackersObjectsAssociation::PostIdAssign()
         进行Id的后分配，得到`post_assignments`,相机检测得到的`unassigned_sensor_objects`在此处进行id分配，得到`post_assignment`,并添加到前面得到的`assignments`中

       - HMTrackersObjectsAssociation::GenerateUnassignedData(）
         剔除`assignments`从而获取未被分配的unassigned_tracks和unassigned_measurements

       - HMTrackersObjectsAssociation::ComputeDistance()
         填充`<AssociationReult>`数据结构中的`track2measurements_dist`和`measurement2track_dist`

     - ProbabilisticFusion::UpdateAssignedTracks
       对于每个track根据当前已匹配的新的测量object更新track的状态，此处有一个参数`match_distance`用于在`ExistanceFusion`中计算`toic score`,由于此处track 和 object已经匹配，直接设置为0与实际的
match_distance差距不大，因此此处设为0
       
       - PbfTracker::UpdateWithMeasurement()
         更新包括4个融合方法：类型融合`DstTypeFusion`,运动状态融合`KalmanMotionFusion`,存在性融合
    `DstExistanceFuison`以及形状外观融合`PbfShapeFusion` 
       
         -  DstExistanceFusion::UpdateWithMeasurement(）
    基于证据的方法更新track存在的可能性
       
           - DstExistanceFusion::GetExistReliability()
      根据测量物体的类型(是否是UNKNOWN)以及测量传感器的类型判断获取存在因子(这里对于lidar,camera,radar的权重依次为0.9,0.8,0.6,radar可信度较低)
       
           - DstExistanceFusion::ComputeDistDecay(）
      将检测物体的中心坐标由世界坐标系转换到传感器坐标系下，并计算距离，若距离大于60m，则设置衰减率为0.8,否则1.0
       
           - Dst::SetBba(）
      设置bba_vec_,根据存在的可信度
       
           - DstExistanceFusion::GetExistanceProbability()
             通过初始化时预先设置的`combination_relations`和上步设置的`bba_vec_`计算
      `probability_vec_`,然后返回`EXIST`对应的概率，计算详细过程见附录。
       
           - `fused_existance_ =   fused_existance_ + existance_evidence * exist_fused_w * association_prob;` 
      通过重构的加法和乘法运算符融合更新`bba_vec_`
       
           - DstExistanceFusion::UpdateToicWithCameraMeasurement(）
      针对测量传感器是相机的情况，基本方法同Existance，先设置`bba_vec_`
       
             - ObjectInCameraView()
               获取物体在相机视野中的可视率in_view_ratio，用于融合更新`bba_vec_` ,三种方法：
               根据与该相机检测物体相匹配的track中的最近检测到的lidar或radar物体点云中的点出现在相机视野中的比例；当没有点云信息时，则根据物体3d边界框的顶点投影到相机图像平面的二维边界框在相机视野中所占的比例确定可视率；若边界框也无效的话，则根据物体三维中心点投影到相机图像平面的二维中心点，判断中心点是否位于相机的视野中，是则可视率直接设为1，否则为0。
        由于相机检测物体随着距离的增加准确率会降低，因此根据下图所示的类sigmoid函数得到检测距离对应的权重(图中所示以150作为相机最大有效检测距离，0.25表示曲线的斜率),距离越远对应的值越小，用该值乘以上面得到的可视率作为最终的结果。
              <img src="apollo-fusion\image-20200512151919053.png" alt="image-20200512151919053" style="zoom:30%;" />
           
          - DstExistanceFusion::UpdateExistanceState()
            根据各自更新的`bba_vec_`，分别得到对应的概率值`GetToicProbability`和
            `GetExistanceProbability`，并赋值到`track`的相关属性中。
            
              更新`track->toic_prob_`和`track->existance_prob_`
            
           - KalmanMotionFusion::UpdateWithMeasurement(）
             基于Kalman Filter更新track的运动信息:
         
           - KalmanMotionFusion::InitFilter()
             初始化滤波器的相关状态和不确定性矩阵，全局状态有
               $[center(2),velocity(2),acceleration(2)]$6个。
         
             -  KalmanMotionFusion::UpdateSensorHistory(）
               `std::deque<base::SensorType> history_sensor_type_`中存储了过去物体测量值对应的传感器类型，存储容量为20，`history_velocity_`,`history_timestamp_`同理，分别存储过去测量值的速度和检测时间戳，用途是计算加速度和pseudo measurement，注意radar和lidar各自的最长追溯值为3个。
             -  KalmanFilter::Init()
               初始化状态矩阵，观测矩阵，状态转移矩阵，状态不确定性，观测不确定性，
               gain_break,value_break,kalman_gain
             -  KalmanFilter::SetGainBreakdownThresh()
               `gain_break_down={0,0,0,0,1,1}  gain_break_down_threshold = 2.0f`
               用于校正不合理的加速度增益
             -  KalmanFilter::SetValueBreakdownThresh()
               `value_break_down={0,0,1,1,0,0} value_break_down_threshold = 0.05f`
                用于校正不合理的速度
         
            - KalmanMotionFusion::MotionFusionWithMeasurement()
         
                 - KalmanFilter::Predict(）
                预测状态X(k+1|k)和协方差阵P `global_state,global_uncertainty`
         
              - KalmanMotionFusion::ComputeAccelerationMeasurement()
                计算加速度，当measurement传感器类型为camera时利用kalman滤波当前推断的加速度状态，当measurement传感器类型为lidar或radar利用`history_velocity`
         
              - 根据measurement object的中心位置和速度的不确定性更新测量协方差矩阵R
         
              -  KalmanMotionFusion::ComputePseudoMeasurement()
         
                  计算伪测量值，根据lidar和radar，camera的原始测量值,以lidar为例，如果给定lidar的测量值能够在短时间内较好的估计radar的测量值，则将radar的测量值投影到给定的lidar测量值，具体如下：
         
                  -  KalmanMotionFusion::ComputePseudoLidarMeasurement()
         
                        以lidar为例,首先追溯radar的测量历史`history_velocity_` ,试图通过找到好的radar测量值用于获得更加精确的lidar测量值(radar传感器本身速度测量较准)，如何算好的radar历史测量值？给出如下标准：
         
                      ```c++
                       1. radar速度的长度(标量)应与lidar速度的长度差值小于阈值5
                       2. radar测量速度矢量与lidar测量速度矢量的夹角(认为时角速度)小于阈值pi/20
                       3. radar速度矢量在lidar速度方向上的投影向量与lidar速度向量的差值矢量与加速度矢量的夹角(认为是角加速度)小于阈值
                       4. 上述投影差值矢量也应小于一定阈值
                      ```
         
                        满足条件，则将radar速度矢量的在lidar方向的投影值作为pseudo_measurement
         
              -  KalmanMotionFusion::RewardRMatrix()
         
                  根据传感器类型和速度是否收敛(object->velocity_converged)调整测量协方差阵R，
         
              - KalmanFilter::DeCorrelation() 去相关
         
              - KalmanFilter::Correct(）根据测量值和测量的不确定性矩阵R校正
         
              -  KalmanFilter::CorrectionBreakdown()
         
                  判断加速度和速度状态的变化是否小于一定阈值，否则速度设为0，加速度的根据阈值进行设置。
         
              - 卡尔曼滤波更新之后进行形状和位置的融合，如果测量传感器是lidar：使用lidar的anchor point和fused velocity。如果测量传感器是radar，同时没有历史的lidar跟踪目标，使用fused anchor point 和fused velocity。如果测量传感器是camera,同时有历史的lidar跟踪目标，使用fused position和fused velocity(滤波器估计值)；测量传感器是camera，同时没有历史lidar跟踪目标但是有radar跟踪目标，位置采用radar的anchor_point,速度采用滤波器的结果fused_velocity()；既没有lidar有没有radar，直接使用测量值作为fused_velocity和fused_anchor_point_。
               加速度的更新因为没有传感器测量直接采用卡尔曼滤波器的状态输出结果。
         
              - KalmanMotionFusion::UpdateMotionState()
                更新`track->fused_object`的运动状态，包括`velocity,acceleration,center_uncertainty,velocity_uncertainty,`
                ,acceleration_uncertainty`,不确定性目前是直接根据measurement赋值
         
         -  PbfShapeFusion::UpdateWithMeasurement(）
         
               - PbfShapeFusion::UpdateState(）
                 - PbfShapeFusion::UpdateShape(）
                   更新`track->fused_object`的DstTypeFusion::UpdateWithMeasurement(形状信息,包括`size,direction,theta,polygon`
                 - PbfShapeFusion::UpdateCenter(）
                 更新`track->fused_object`的中心坐标，包括`center,anchor_point`,均直接根据测量值更新，没有中间处理
               
         - DstTypeFusion::UpdateWithMeasurement()
           创建测量值对应的`Dst`
         
           - DstTypeFusion::TypeProbsToDst()
             将物体`object->type_probs`转换为Dst。
         
             - DstTypeFusion::TypToHyp()
               将物体原检测类型转换为`Dst hypothesis types`,映射关系为:
         
               | origin object type | value | dst hypothesis types | value          |
               | ------------------ | ----- | -------------------- | -------------- |
               | UNKNOWN            | 0     | OTHERS               | 0001 1000 (24) |
               | UNKNOWN_MOVABLE    | 1     | OTHERS_MOVABLE       | 0000 1000 (8)  |
               | UNKNOWN_UNMOVABLE  | 2     | OTHERS_UNMOVABLE     | 0001 0000 (16) |
               | PEDESTRIAN         | 3     | PEDESTRIAN           | 0000 0001 (1)  |
               | BICYCLE            | 4     | BICYCLE              | 0000 0010 (2)  |
               | VEHICLE            | 5     | VEHICLE              | 0000 0100 (4)  |
               |                    |       | UNKNOWN              | 0001 1111 (31) |
         
               将`origin object type`的检测概率值分别赋值到对应的`dst type`中构成`res_bba_map` 
         
             - Dst::SetBba()
               设置`bba_vec_`根据当前检测类别概率值构造的`res_bba_map`,`fod_subsets_`包括:
               `{PEDESTRIAN, BICYCLE, VEHICLE, OTHERS_MOVABLE, OTHERS_UNMOVABLE, OTHERS,  UNKNOWN}` 
         
           - 根据 当前测量值设置`bba_vec_` 的`measurement_dst`对之前存在的`fused_dst_`进行更新。
         
             ```c++
             fused_dst_ =
                   fused_dst_ + measurement_dst * GetReliability(measurement->GetSensorId());
             ```
         
           - DstTypeFusion::GetReliability()
             获取当前检测传感器对于物体类别的可靠程度：
             `velodyne64:0.5,front_6mm:0.95,front_12mm:0.5,....`等等,相机的类别检测的置信度较高
             
           - 如果当前测量值对应的传感器是相机，则将测量值对应的`object->sub_type`赋值到
             `fused_object->sub_type`
         
           - DstTypeFusion::UpdateTypeState()
             根据融合后的`bba_vec_`更新`fused_object`的类型概率。
         
             - DstTypeFusion::HypToTyp()
               将`hypothesis type`映射回原物体类型，这里将最大概率(bba_vec_中最大值索引)的`hypothesis`映射回`origin object type`并设置为`fused_object->type`
         
             - 修正`fused_object->subtype`
               之前`fused_object->sub_type`由测量的物体的`sub_type`直接赋值，此处在更新了`type`之后，需要判断该子类型对应的`type`是否和更新的`type`相等，不相等将`fused->subtype`设置为`base::ObjectSubType::UNKNOWN`
         
             - 更新`fused_object->type_probs`
         
               根据`fused_subset`与原`type`的映射，将`bba_vec_`中的值复制到`type_probs`中
               
         
         - Track::UpdateWithSensorObject()
            更新track对应的`SensorObject`,添加带有传感器id标记的object到对应的`lidar_objects_`,`radar_objects_`或`camera_objects_`的map中进行覆盖。然后将map中超时未被测量值匹配的object删除。同时需要更新最近一次被跟踪的时间戳为当前测量值的时间戳，更新对应传感器类型的补充属性，设置`is_alive_=True`，表明该track为活跃的。
         
            - Track::UpdateSensorObject()
              更新测量值对应传感器类型的objects中对应sensor_id(用于区分同一类传感器)的object,注意此处为直接覆盖。
            - Track::UpdateSensorObjectWithMeasurement()
              根据传感器最大容忍的invisible时间间隔(lidar 0.25s radar 0.5s camera 0.75s),超出该时间间隔(即在这段时间内没有新的测量值进行匹配)则从objects中将对应的object删除
              对track中的`lidar_objects`,`radar_objects`,`camera_objects`分别执行上述过程。
            - 更新`fused_object_->latest_tracked_time`为当前测量物体的时间戳。
            - Track::UpdateSupplementState()
              将测量值`object`的补充属性直接赋值到`fused_object`的补充属性中。同时对于track中
              `lidar_objects_`,`radar_objects_`或`camera_objects_`谁为空(目前该传感器还没有匹配的测量值)就将谁对应的`fused_object`中对应的`lidar_supplement`,`camera_supplement`或
              `camera_supplement`重置
            - Track::UpdateUnfusedState()
              更新未融合的`fused_object`剩下的相关状态，对于lidar更新`fused_object->confidence`和`fused_object->velocity_converged`,对于radar没有啥还需要更新的，对于camera需要更新`fused_object->confidence`
       
     - ProbabilisticFusion::UpdateUnassignedTracks()
       更新未被当前帧测量值匹配的track的状态,与函数`UpdateAssignedTracks()`类似，此处有一个参数`match_distance`用于在`ExistanceFusion`中计算`toic score`,由于此处track没有对应的测量值进行匹配，由于没有用到，临时设置match_dsitance为0。
     
       - PbfTracker::UpdateWithoutMeasurement()
         更新track的状态同样也包含四类融合：类型融合`DstTypeFusion`,运动状态融合`KalmanMotionFusion`,存在性融合`DstExistanceFuison`以及形状外观融合`PbfShapeFusion`,基本方法同有测量值的更新，此处简述：
     
         - DstExistanceFusion::UpdateWithoutMeasurement()
           输入:包括sensor_id,测量时间戳(当前检测数据帧的时间戳),`min_match_dist`(输入时的临时值为0)
           输出:`track->toic_prob`,`track->existance_prob`
           
           - DstExistanceFusion::UpdateToicWithoutCameraMeasurement()
             虽然当前帧没有匹配的测量值,但是利用历史track最近匹配lidar或radar检测物体进行可视程度的判断,用于更新`toic_`
           
         - KalmanMotionFusion::UpdateWithoutMeasurement()
           就是根据时间差异进行一个简单的预测，由于没有测量因此也就没有校正阶段,但预测的前提是要求该track已经的kalman_filter已经初始化过同时有过匹配的测量值,更新fused_object的相关属性如下：
           `track->fused_object->velocity`,`acceleration`,`center_uncertainty`,
           `velocity_uncertainty`,`acceleration_uncertainty`
           
         - PbfShapeFusion::UpdateWithoutMeasurement()
           do nothing!
           
         - DstTypeFusion::UpdateWithoutMeasurement()
           仅对当前帧的测量传感器是相机的情况进行处理,比较特殊的是相机在计算不可信度时，采用了时间信息，根据当前测量的时间戳转换到小时，判断是否是晚上以此获得更新的权重比例。
           
           ```c++
             std::map<std::string, double> sensor_reliability_for_unknown_ = {
               {"velodyne64", 0.5},          {"velodyne_64", 0.5},
               {"velodyne128", 0.5},         {"camera_smartereye", 0.2},
               {"front_6mm", 0.2},           {"camera_front_obstacle", 0.2},
               {"camera_front_narrow", 0.2},
           ```
           
           更新track的相关属性如下:
           `track->fused_object->type_probs`
           
         - Track::UpdateWithoutSensorObject()
           
           - Track::UpdateSensorObjectWithoutMeasurement()
             根据当前测量时刻的时间戳对track目前存在的历史`lidar_objects`,`radar_objects`
             ,`camera_objects`的`invisible_period`进行更新。对于`invisible_period`大于阈值的
             objects进行清除。
             根据track当前的`lidar_objects,radar_objects`,`camera_objects`是否全部为空来判断track是否是alive。
     
     - ProbabilisticFusion::CreateNewTracks
     
       根据未进行匹配当前帧测量物体,创建新的跟踪序列。
     
       - Track::Initialize()
         从并发对象池中获取track实例，并进行初始化，初始化过程主要由以下几步：
         1. 重置`lidar_objects_,radar_objects_,camera_objects_`, 将fused_object设置为当前测量值并分配新的全局track_id。
         2. Track::UpdateWithSensorObject()根据测量值对应的sensor_id将相应值添加到`lidar_objects_`或
            `radar_objects`或`camera_objects`中
       - Scene::AddForegroundTrack(TrackPtr track)
         将该`track_`添加到`foreground_tracks_`列表中。
       - PbfTracker::Init(TrackPtr track, SensorObjectPtr measurement)
         创建`<PbfTracker>`实例`tracker`，并通过`track_`和测量值进行初始化,首先通过`InitMethods`初始化跟踪融合的相关方法`type_fuison,shape_fusion,existance_fusion,motion_fusion`,创建他们的方法类实例。
       - 将创建的`tracker`添加到`ProbabilisticFusion::trackers_`列表中
         
     
   - ProbabilisticFusion::FusebackgroundTrack(const SensorFramePtr& frame)
     背景物体跟踪,注意只有lidar传感器检测背景类物体。
     - 首先进行background_tracks和background_objects的关联，关联依据是track->fused_object->track_id(即该track目前对应的object的局部track_id)与当前检测物体的局部`track_id`相等，则进行关联
     - 然后对已经匹配的track和object进行状态更新:
       Track::UpdateWithSensorObject(const SensorObjectPtr& obj)
       更新`track`中的sensor_objects (此处应该为`lidar_objects`)
       - Track::UpdateWithSensorObjectForBackground()
         根据测量值的BaseObject更新`fused_object_`的BaseObject,但是保留原`fused_object_`的track_id
     - 然后对未匹配的track进行状态更新:
       Track::UpdateWithoutSensorObject()
     - 对未匹配的object创建新的跟踪序列:
       从对象池中获取`track`实例，进行初始化`track->Initialize(...);`分配新的track_id。 
       - Scene::AddBackgroundTrack()
         将新创建的track加入到背景跟踪列表`background_tracks_`中。
   - ProbabilisticFusion::RemoveLostTrack() 
     对应前景类，移除`track`和track对应的处理类`tracker`(仅前景类foreground存在该tracker处理)
     查看`track->is_alive_`是否为true(本质上为track的lidar_objects_,radar_objects,camera_objects是否全为空),
     对于背景类，仅需要移除`track`
   
4. **collect fused objects**

   ```c++
    // @brief: 收集融合的检测目标
    // @params[in]: timestamp 当前传感器帧的时间戳
    // @params[out]: fused_objects 融合的objects
    void ProbabilisticFusion::CollectFusedObjects(double timestamp,
                              std::vector<base::ObjectPtr>* fused_objects);
   ```

   - PbfGatekeeper::AbleToPublish(const TrackPtr &track)
     对于前景物体跟踪的各个track查询各个类别传感器对应的`invisible_period_`,下列三个函数返回各类传感器是否是Visible

     ```c++
     track->IsLidarVisible() //(note:lidar类别传感器有不同的id,例如64线,32线等等)
     track->IsRadarVisible() //相机和radar同理
     track->IsCameraVisible()
     ```

     判断`invisible_period< 1.0e-6`(要求一类传感器中的所有object) 即判断该传感器是否存在跟踪的中断。
     下面三个函数分别判断传感器数据能否发布,若三种传感器都不能发布则返回False。

     ```c++
     //要求lidar is visible 同时参数params_.publish_if_has_lidar为true时返回True
     PbfGatekeeper::LidarAbleToPublish()
     //要求radar is visible 同时参数params_.publish_if_has_radar为true时,若track->radar_object
     // 为"radar_front" ,返回false;若为"radar_rear",则满足：
     // track->radar_object.range > 40; track->radar_object.velocity>4.0,
     // track->existance_prob_>0.9 时返回true.
     PbfGatekeeper::RadarAbleToPublish()
     //　要求camera is visible 同时params_.publish_if_has_camera为true,is_night=False(白天)。
     //  当subtype=TRAFFICCONE无视range大小及进行发布；若subtype=UNKNOWN_UNMOVABLE则还要求物体的
     //  距离相机的距离>50
     //　track->existance_prob_>0.7
     PbfGatekeeper::CameraAbleToPublish()
     ```

     - AddTrackedTimes()
       ++tracked_times_　增加该track被跟踪次数。 前提是上述条件满足。
     - GetTrackedTimes()
       紧接着查询正常跟踪的次数，如果大于预设参数`params_.pub_track_time_thresh=3`表明该track已经超过3次被正常跟踪，可以进行发布了。

   - ProbabilisticFusion::CollectObjectsByTrack()
     从跟踪tracks中收集objects的相关属性,引入数据类型`<SensorObjectMeasurement>`位于object->fusion_supplement中:

     ```c++
       std::string sensor_id = "unknown_sensor";
       double timestamp = 0.0;
       int track_id = -1;
       Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
       float theta = 0.0f;
       Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
       Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
       ObjectType type = ObjectType::UNKNOWN;
       // @brief only for camera measurement
       BBox2D<float> box;
     ```

     - ProbabilisticFusion::CollectSensorMeasurementFromObject(...)

       首先对于基础属性，直接将track->fused_object->baseobject赋值到该obj中;
       对于补充属性(fusion_supplement),从track->lidar_objects,track->radar_objects,track->camera_objects中分别得到上述`<SensorObjectMeasurement>`中的参数值，保存到` obj->fusion_supplement.measurements`中，其中`measurements`为一个向量数组，大小为下所示，分别保存了对应该track的所有测量值。
       `lidar_objects_.size() + camera_objects_.size() + radar_objects_.size()`
       除了上述数据类型中的值外,设置该obj的track_id,latest_tracked_time(上一次被跟踪的时间),tracking_time(跟踪时长)

       ```c++
         obj->track_id = track->GetTrackId();
         obj->latest_tracked_time = timestamp;
         obj->tracking_time = track->GetTrackingPeriod();
       ```

       对每一个track执行上述操作后，得到的obj添加到`fused_objects`列表中。

   - 对于背景物体的跟踪的相关处理方法同上，此处不赘述。

至此，`fusion_->Fuse(options, frame, objects)`完成,返回到`fusion_component`组件中。

------

序列化消息用于发布：`viz_message`,`out_message`
当测量传感器为主传感器时，通过节点发布。






### 参考链接

https://developer.baidu.com/topic/show/290445

https://blog.csdn.net/u012423865/article/details/80386444

https://zhuanlan.zhihu.com/p/116901401

https://blog.csdn.net/zhanghm1995/article/details/104910923

视锥体，判断Lidar 3D点是否位于相机视野内：
https://blog.csdn.net/qq_29797957/article/details/96914792

### 补充

#### 1.卡方分布：
由n个相互独立的随机变量，均服从标准正态分布(独立同分布于标准正太分布)，则这n个服从标准正态分布的随机变量的平方和构成新的随机变量，其分布规律称为卡方分布，卡方分布位于第一象限，均值为自由度v，方差为2v。

#### 2. apollo中`GateHungrianMatcher`的匹配流程：

1. 创建优化器实例：

   ```c++
   common::GatedHungarianMatcher<float> optimizer_;
   ```

   通过实现已经获取的度量矩阵设置`global_costs`

   ```c++
   common::SecureMat<float>* global_costs = optimizer_.mutable_global_costs();
   ```

2. 运行主接口函数：

   ```c++
   optimizer_.Match(static_cast<float>(s_match_distance_thresh_), // thresh 4.0
                      static_cast<float>(s_match_distance_bound_), opt_flag, //bound 100.0
                      &local_assignments, &local_unassigned_tracks, //row -> tracks
                      &local_unassigned_measurements); ///column -> measurements
   ```

   **2.1** `MatchInit()` 获取行数和列数(即跟踪目标数量rows和测量目标cols的数量)，并根据opt_flag，确定比较函数,此处包含的选项有`OptimizeFlag::OPTMAX `和`OptimizeFlag::OPTMIN`分别对应与`thresh`的比较方式,例如当`opt_flag=OPTMIN`时，当`global_cost(i,j)<s_match_distance_thresh_`时返回ture。
   **2.2**  `ComputeConnectedComponents(&row_components, &col_components)` 为了加速匹配的速度，将原始的cost_graph分割为几个小的子二分图,根据阈值`s_match_distance_thresh_`判断连接是否有效，将有效的相互连接组成一个
   `Component` 以此将原始图进行分割为独立的子图，下图所示为关联矩阵以2.1为阈值，并进行图分割为2个component的例子，0~6为对应的节点id，其中0~2对应行,3~6对应列(id=row_num+col_id),最终函数返回：
   `row_components[0]={0,1},row_components[1]={2}；col_components[0]={3,4},col_components[1]={5,6}` 

   <img src="apollo-fusion\image-20200510203006092.png" alt="image-20200510203006092" style="zoom:30%;" />            

   **2.3** `OptimizeConnectedComponent` 对每个component进行优化，进行分配。
   **2.4** `GenerateUnassiganedData` 生成未分配的节点的集合

#### 3. PbfTracker的更新融合方法

##### 3.1 DstExistanceFusion

```c++
struct ExistanceDstMaps {
  enum { EXIST = (1 << 0), NEXIST = (1 << 1), EXISTUNKOWN = (EXIST | NEXIST) }; //1,2,3
  std::vector<uint64_t> fod_subsets_ = {EXIST, NEXIST, EXISTUNKOWN};
  std::vector<std::string> subset_names_ = {"EXIST", "NEXIST", "EXISTUNKOWN"};
  //EXISTUNKOWN从二进制角度来说包含了EXIST,NEXIST
  //subsets的索引ind是指从0开始各个subset的编号，此处 0 1 2 
  //它们的实际值为 01  10  11
};
```

基于`<Dst>`数据结构，重构了加法和乘法运算符，对应成员变量：

```c++
  std::string app_name_; //唯一的标识符
  mutable DstCommonDataPtr dst_data_ptr_ = nullptr; //由DstManager管理的DstCommonData
  mutable std::vector<double> bba_vec_; //大小为subset的大小，内容为各个subset的mass(可理解为概率)取值0~1
  mutable std::vector<double> support_vec_;
  mutable std::vector<double> plausibility_vec_; //合理性
  mutable std::vector<double> uncertainty_vec_; //不确定性
  mutable std::vector<double> probability_vec_; //大小为bba_vec的大小
```

```c++
struct DstCommonData {
  // ensure initialize DSTEvidence once
  bool init_ = false;
  // fods
  size_t fod_loc_ = 0; //指向subsets的最后一个元素对应的编号值
  std::vector<uint64_t> fod_subsets_;
  // for transforming to probability effectively 存储各subset的二进制中1的位数
  std::vector<size_t> fod_subset_cardinalities_;
  std::vector<std::string> fod_subset_names_;
  // for combining two bbas effectively. 用于计算probability_vec_,
  //  第一维表示两个fod_subset交集(大小为fod_sets大小)对应的subset的索引编号
  //  第二维为对应该交集的所有两个fod_subset的索引编号pair
  //　用于结合两个bbas
  std::vector<std::vector<std::pair<size_t, size_t>>> combination_relations_; 
  // for computing support vector effectively,第一维大小为fod_subsets大小，第二维存储各fod_subset二进制
  //包含（存在包含关系）的subset所对应的索引编号
  std::vector<std::vector<size_t>> subset_relations_;
  // for computing plausibility vector effectively，第一维大小为fod_subsets大小，第二位存储各fod_subset二
  // 进制具有相同二进制位(存在交集)的subset对应的索引编号
  std::vector<std::vector<size_t>> inter_relations_; //intersection
  std::map<uint64_t, size_t> subsets_ind_map_;  //pair(subset,ind)
};
```



概率计算举例：

```c++
struct ExistanceDstMaps {
    enum { EXIST = (1 << 0), NEXIST = (1 << 1), EXISTUNKOWN = (EXIST | NEXIST) };
    std::vector<uint64_t> fod_subsets_ = {EXIST, NEXIST, EXISTUNKOWN};
    std::vector<std::string> subset_names_ = {"EXIST", "NEXIST", "EXISTUNKOWN"};
};
std::vector<double> probability_vec_;
std::vector<double> bba_vec_={0.7,0,0.3};
std::vector<std::vector<std::pair<size_t ,size_t >>> combination_relations_;
std::vector<size_t> fod_subset_cardinalities_={1,1,2};
void ComputeProbability()  {
    combination_relations_.resize(bba_vec_.size());

    combination_relations_[0].push_back(std::make_pair(0,0));
    combination_relations_[0].push_back(std::make_pair(0,2));
    combination_relations_[0].push_back(std::make_pair(2,0));
    combination_relations_[1].push_back(std::make_pair(1,1));
    combination_relations_[1].push_back(std::make_pair(1,2));
    combination_relations_[1].push_back(std::make_pair(2,1));
    combination_relations_[2].push_back(std::make_pair(2,2));

    probability_vec_.clear();
    probability_vec_.resize(bba_vec_.size(), 0.0);
    const auto &combination_relations = combination_relations_; //存储两个subset的交集索引编号和该两个subset的编号
    const std::vector<size_t> &fod_subset_cardinalities =
            fod_subset_cardinalities_; //存储各个subset对应的二进制值中1的个数
    for (size_t i = 0; i < combination_relations.size(); ++i) {
        const auto &combination_pairs = combination_relations[i];//交集为各个subset的pair
        double intersection_card = static_cast<double>(fod_subset_cardinalities[i]); //该subset对应的位数
        for (auto combination_pair : combination_pairs) {
            size_t a_ind = combination_pair.first;
            size_t b_ind = combination_pair.second;
            probability_vec_[a_ind] +=
                    intersection_card /
                    static_cast<double>(fod_subset_cardinalities[b_ind]) *
                    bba_vec_[b_ind];//根据交集的位数占后者的位数的比例确定所占概率比例的大小
        }
    }
}

//@ result：
probability_vev_={0.85,0.15,1}
//@ note: bba_vec_={0.7,0,0.3}
//@ 理解为此处的EXIST，NEXIST均与EXISTUNKOWN有交集，因此根据bba_vec按照交集位数的比例比如NEXIST与EXISTUNKOWN的二进制位数为1的比例为 1/2=0.5，因此其概率为0.3*0.5=0.15,而EXIST为0.7(自身概率)+0.3*0.5=0.85
```

更新概率：
通过重构运算符加法和乘法，进行概率融合，

```c++
Dst operator*(const Dst &dst, double w) {
  dst.SelfCheck();
  Dst res(dst.app_name_);
  // check w
  if (w < 0.0 || w > 1.0) {
    AERROR << boost::format(
                  "the weight of bba %lf is not valid, return default bba") %
                  w;
    return res;
  }
  size_t fod_loc = dst.dst_data_ptr_->fod_loc_;
  std::vector<double> &resbba_vec_ = res.bba_vec_;
  const std::vector<double> &bba_vec = dst.bba_vec_;
  for (size_t i = 0; i < resbba_vec_.size(); ++i) {
    if (i == fod_loc) {
      resbba_vec_[i] = 1.0 - w + w * bba_vec[i]; //???
    } else {
      resbba_vec_[i] = w * bba_vec[i];
    }
  }
  return res;
}
```



##### 3.2 DstTypeFusion

```c++
struct DstMaps {
  // dst hypothesis types
  enum {
    PEDESTRIAN = (1 << 0), //1      0000 0001
    BICYCLE = (1 << 1), //2         0000 0010
    VEHICLE = (1 << 2), //4         0000 0100
    OTHERS_MOVABLE = (1 << 3), //8  0000 1000
    OTHERS_UNMOVABLE = (1 << 4)//16 0001 0000   
  };
  enum {
    OTHERS = (OTHERS_MOVABLE | OTHERS_UNMOVABLE), //24 0001 1000
    UNKNOWN = (PEDESTRIAN | BICYCLE | VEHICLE | OTHERS) //31 0001 1111
  };

  std::vector<uint64_t> fod_subsets_ = {
      PEDESTRIAN,       BICYCLE, VEHICLE, OTHERS_MOVABLE,
      OTHERS_UNMOVABLE, OTHERS,  UNKNOWN};
  std::vector<std::string> subset_names_ = {
      "PEDESTRIAN",       "BICYCLE", "VEHICLE", "OTHERS_MOVABLE",
      "OTHERS_UNMOVABLE", "OTHERS",  "UNKNOWN"};
  std::unordered_map<size_t, uint64_t> typ_to_hyp_map_ = {
      {static_cast<size_t>(base::ObjectType::PEDESTRIAN), PEDESTRIAN},
      {static_cast<size_t>(base::ObjectType::BICYCLE), BICYCLE},
      {static_cast<size_t>(base::ObjectType::VEHICLE), VEHICLE},
      {static_cast<size_t>(base::ObjectType::UNKNOWN_MOVABLE), OTHERS_MOVABLE},
      {static_cast<size_t>(base::ObjectType::UNKNOWN_UNMOVABLE),
       OTHERS_UNMOVABLE},
      {static_cast<size_t>(base::ObjectType::UNKNOWN), OTHERS},
  };
  std::map<uint64_t, size_t> hyp_to_typ_map_ = {
      {PEDESTRIAN, static_cast<size_t>(base::ObjectType::PEDESTRIAN)},
      {BICYCLE, static_cast<size_t>(base::ObjectType::BICYCLE)},
      {VEHICLE, static_cast<size_t>(base::ObjectType::VEHICLE)},
      {OTHERS_MOVABLE, static_cast<size_t>(base::ObjectType::UNKNOWN_MOVABLE)},
      {OTHERS_UNMOVABLE,
       static_cast<size_t>(base::ObjectType::UNKNOWN_UNMOVABLE)},
      {OTHERS, static_cast<size_t>(base::ObjectType::UNKNOWN)},
      {UNKNOWN, static_cast<size_t>(base::ObjectType::UNKNOWN)}};
};
```



##### 3.3 KalmanMoitonFusion

##### 3.4 PbfShapeFusion





#### 4. 自适应卡尔曼滤波