---
title: apollo lidar算法--recognition component（一）
tags:
- lidar目标跟踪
categories:
- apollo
mathjax: true
---

该组件实现lidar的目标跟踪算法，该组件接收segment得到的检测物体信息，然后建立帧与帧之间的联系以实现帧间同一目标速度等动态信息的预测。

`Init->Preprocess->MapManager->Segmentaion->ObjectBuilder->ObjectFilter->Classifier->Tracker`

<!--more-->

## 一、 组件初始化和配置

### 1. 外部参数配置

```protobuf
# 组件的dag文件配置
components {
    class_name : "RecognitionComponent"
    config {
      name: "RecognitionComponent"
      config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/recognition_conf.pb.txt"
      readers {
          channel: "/perception/inner/SegmentationObjects"
        }
    }
  }
```

组件的配置参数：

```protobuf
# `recognition_conf.pb.txt`
main_sensor_name: "velodyne64"
output_channel_name: "/perception/inner/PrefusedObjects"
```

### 2. 初始化

创建writer的通道名称为`/preception/inner/PrefusedObjects`，消息类型为:`<SensorFrameMessage>` 与相机最终的输出预融合类型相同。
然后进行算法组件的初始化：
使用的lidar算法组件类`<LidarObstacleTracking>`

| 参数名称             | 默认值          | 参数说明           | 参数类型 |
| -------------------- | --------------- | ------------------ | -------- |
| multi_target_tracker | MlfEngine       | 多目标跟踪器的名称 | Class    |
| fusion_classifier    | FusedClassifier | 分类器的名称       | Class    |

**multi_target_tracker（多雷达融合）进行初始化：**

```c++
  bool Init(const MultiTargetTrackerInitOptions& options =
                MultiTargetTrackerInitOptions()) override;
```

参数：`multi_lidar_fusion.config` -> `multi_lidar_fusion/mlf_engine.conf`

| 参数名称                | 默认值 | 参数说明 | 参数类型 |
| ----------------------- | ------ | -------- | -------- |
| use_histogram_for_match | true   |          |          |
| histogram_bin_size      | 10     |          |          |
| output_predict_objects  | false  |          |          |
| reserved_invisible_time | 0.3    |          |          |
| use_frame_timestamp     | true   |          |          |

参数初始化完成后，进行`tracker_`,`matcher_` 的初始化
`matcher_`的功能实现类为`<MlfTrackObjectMatcher>` 

| 参数名称                  | 默认值                         | 参数说明 | 参数类型 |
| ------------------------- | ------------------------------ | -------- | -------- |
| foreground_matcher_method | "MultiHmBipartiteGraphMatcher" |          |          |
| background_matcher_method | "GnnBipartiteGraphMatcher"     |          |          |
| bound_value               | 100                            |          |          |
| max_match_distance        | 4.0                            |          |          |

`track_object_distance_`的功能实现类`<MlfTrackObjectDistance>`

| 参数名称                   | 默认值 | 参数说明 | 参数类型 |
| -------------------------- | ------ | -------- | -------- |
| location_dist_weight       | “ ”    |          |          |
| direction_dist_weight      | 0      |          |          |
| bbox_size_dist_weight      | 0      |          |          |
| point_num_dist_weight      | 0      |          |          |
| histogram_dist_weight      | 0      |          |          |
| centroid_shift_dist_weight | 0      |          |          |
| bbox_iou_dist_weight       | 0      |          |          |

 `tracker_`的功能实现类为`<MlfTracker>`

| 参数名称    | 默认值            | 参数说明 | 参数类型 |
| ----------- | ----------------- | -------- | -------- |
| filter_name | "MlfShapeFilter"  |          |          |
| filter_name | "MlfMotionFilter" |          |          |

`MlfShapeFilter`初始化：

| 参数名称                       | 默认值 | 参数说明 | 参数类型 |
| ------------------------------ | ------ | -------- | -------- |
| bottom_points_ignore_threshold | 0.1    |          |          |
| top_points_ignore_threshold    | 1.6    |          |          |

`MlfMotionFilter`初始化：

| 参数名称                     | 默认值 | 参数说明 | 参数类型 |
| ---------------------------- | ------ | -------- | -------- |
| use_adaptive                 | true   |          |          |
| use_breakdown                | true   |          |          |
| use_convergence_boostup      | true   |          |          |
| init_velocity_variance       | 5.0    |          |          |
| init_acceleration_variance   | 0.6    |          |          |
| measured_velocity_variance   | 0.6    |          |          |
| predict_variance_per_sqrsec  | 50.0   |          |          |
| boostup_history_size_minimum | 3      |          |          |
| boostup_history_size_maximum | 6      |          |          |
| converged_confidence_minimum | 0.5    |          |          |
| noise_maximum                | 0.1    |          |          |
| trust_orientation_range      | 40     |          |          |

`<MlfMOtionRefiner>`的参数初始化：

| 参数名称                       | 默认值 | 参数说明 | 参数类型 |
| ------------------------------ | ------ | -------- | -------- |
| claping_acceleration_threshold | 10     |          |          |
| claping_speed_threshold        | 1      |          |          |

**fusion_classifier 初始化**
`<FusedClassifier>`参数：`../data/.../fused_classifier/fused_classifier.conf`

| 参数名称               | 默认值                   | 参数说明 | 参数类型 |
| ---------------------- | ------------------------ | -------- | -------- |
| one_shot_fusion_method | "CCRFOneShotTypeFusion"  |          |          |
| sequence_fusion_method | "CCRFSequenceTypeFusion" |          |          |
| enable_temporal_fusion | true                     |          |          |
| temporal_window        | 20.0                     |          |          |
| use_tracked_objects    | true                     |          |          |

`<CCRFOneShotTypeFusion>` 类参数：

| 参数名称                       | 默认值                                     | 参数说明 | 参数类型 |
| ------------------------------ | ------------------------------------------ | -------- | -------- |
| classifiers_property_file_path | "../fused_classifier/classifiers.property" |          | 数字矩阵 |
|                                |                                            |          |          |
| transition_matrix_alpha        | 1.8                                        |          |          |
| confidence_smooth_matrix_      |                                            |          |          |
| smooth_matrices_               |                                            |          |          |

`<CCRFSequenceTypeFusion>`类参数：

| 参数名称                      | 默认值                                    | 参数说明 | 参数类型 |
| ----------------------------- | ----------------------------------------- | -------- | -------- |
| transition_property_file_path | "../fused_classifier/transition.property" |          | 数字矩阵 |
|                               |                                           |          |          |

---

## 二、算法处理流程Proc

入口函数：

```c++
LidarProcessResult Process(const LidarObstacleTrackingOptions& options,
                           LidarFrame* frame);
```

内部主要的两个处理函数：

```c++
multi_target_tracker_->Track(tracker_options, frame);
...
fusion_classifier_->Classify(fusion_classifier_options, frame);
```

### Track

主入口函数：

```c++
  // @brief: track segmented objects from multiple lidar sensors
  // @params [in]: tracker options
  // @params [in/out]: lidar frame
  bool MlfEngine::Track(const MultiTargetTrackerOptions& options,
             LidarFrame* frame) override;
```

1. modify objects timestamp  if necessary
   将物体检测的时间戳修改为数据帧frame的时间戳，之前`object->latest_tracked_time`为物体点云所有点的平均时间

2. add global offset to pose(only when no track exists)

3. 分离前景和背景的物体，并转换为被跟踪物体

   ```c++
     // @brief: split foreground/background objects and attach to tracked objects
     // @params [in]: objects
     // @params [in]: sensor info
     // @output :background_objects_,foreground_objects_,shape_features,shape_features_full
     void MlfEngine::SplitAndTransformToTrackedObjects(
         const std::vector<base::ObjectPtr>& objects,
         const base::SensorInfo& sensor_info);
   ```
   
   - 向被跟踪列表中添加object：
   
     ```
     // @brief: add object to tracked list
     // @params[in]: objectptr,pose(world->lidar),sensor_info
     
     tracked_objects[i]->AttachObject(objects[i], sensor_to_local_pose_,
                                    global_to_local_offset_, sensor_info);
     ```
   
      向`<TrackedObject>`数据类型中传递object的相关属性(朝向,大小,中心,)。
      **存疑：这个sensor_to_local_pose_的转换方向不明**
   
   - 如果object不是背景且采用直方图匹配方法,则计算物体的外观特征中的`histogram_distance`
   
     ```c++
     tracked_objects[i]->ComputeShapeFeatures();
     ```

- ​    
  
     - 计算物体的外观特征：
     
       ```c++
     // @brief: compute object's shape feature
       // @params[in]: histogram_bin_size default:10
       // @params[out]: object's shape feature
       FeatureDescriptor::ComputeHistogram(int bin_size, float* feature) {...}
       ```
     
       根据object的点云计算特征向量的组成为：
     
       ```c++
          ...　
           feature[0] = center_pt_.x / 10.0f; //x轴点云中心点
           feature[1] = center_pt_.y / 10.0f; //y轴点云中心点
           feature[2] = center_pt_.z; //z轴点云中心点 
           feature[3] = xsize; //x轴点云长度
           feature[4] = ysize; //y轴点云长度
           feature[5] = zsize; //z轴点云长度
           feature[6] = static_cast<float>(pt_num); //点云中点数量
           for (size_t i = 0; i < stat_feat.size(); ++i) {
             feature[i + 7] = //直方图特征,每一个点对应stat_feat中的区间位置
                 static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num);
           } //总共37维
       ```

4. assign tracked objects to tracks,匹配objects和tracks

   ```c++
   // @brief: match tracks and objects and object-track assignment
   // @params [in]: match options  default:null
   // @params [in]: objects for match :foreground and background separately
   // @params [in]: name (foreground or background)
   // @params [in/out]: tracks for match and assignment
   // @note :分开单独处理前景和背景
   void MlfEngine::TrackObjectMatchAndAssign(
       const MlfTrackObjectMatcherOptions& match_options,
       const std::vector<TrackedObjectPtr>& objects, const std::string& name,
       std::vector<MlfTrackDataPtr>* tracks) {...}
   ```

   - **二分图**匹配检测到的detected objects和tracks
     `  matcher_->Match(match_options, objects, *tracks, &assignments,                  &unassigned_tracks, &unassigned_objects);`　

     ```c++
       // @brief: match detected objects to tracks
       // @params [in]: new detected objects for matching
       // @params [in]: maintaining tracks for matching
       // @params [out]: assignment pair of object & track
       // @params [out]: tracks without matched object
       // @params [out]: objects without matched track
       void MlfTrackObjectMatcher::Match(const MlfTrackObjectMatcherOptions &options,
                  const std::vector<TrackedObjectPtr> &objects,
                  const std::vector<MlfTrackDataPtr> &tracks,
                  std::vector<std::pair<size_t, size_t> > *assignments,
                  std::vector<size_t> *unassigned_tracks,
                  std::vector<size_t> *unassigned_objects);
     ```

     - 前景和背景分开处理,此处仅以前景为例,计算关联的代价矩阵:

       ```c++
       // @brief: compute association matrix
       // @params [in]: maintained tracks for matching
       // @params [in]: new detected objects for matching
       // @params [out]: matrix of association distance
       void MlfTrackObjectMatcher::ComputeAssociateMatrix(
           const std::vector<MlfTrackDataPtr> &tracks,
           const std::vector<TrackedObjectPtr> &new_objects,
           common::SecureMat<float> *association_mat) {...}
       ```

       - 计算new detected object 与已存在的track两两之间的匹配程度(距离)`track_object_distance`

         ```c++
         // @brief: compute object track distance
         // @params [in]: object
         // @params [in]: track data
         // @return: distance
         float MlfTrackObjectDistance::ComputeDistance(
             const TrackedObjectConstPtr& object,
             const MlfTrackDataConstPtr& track) const {...}
         ```

         关联特征权重表：

         |          关联特征          | 一致性评估 | 默认权重(前景,背景) |
         | :------------------------: | :--------: | :-----------------: |
         |  **location_dist_weight**  |    运动    |    **0.6** , 0.0    |
         |   direction dist weight    |    运动    |      0.2 , 0.0      |
         |   bbox size dist weight    |    外观    |      0.1 , 0.0      |
         |   point num dist weight    |    外观    |      0.1 , 0.0      |
         | **histogram dist weight**  |    外观    |    **0.5** , 0.0    |
         | centroid shift dist weight |    外观    |      0.0 , 0.2      |
         |    bbox iou dist weight    |    外观    |      0.0 , 0.8      |

         由上表可以看出，在计算关联距离时，重点考虑的是几何距离和两者的形状相似度。

         - `track->PredictState(current_time)`预测状态，包含了位置`latest_anchor_point`和速度信息`latest_velocity` 共6个状态。

         
         根据当前object和track预测的状态信息，分别计算上述7个特征,加权求和，函数实现位于
         `distace_collection.h`中。

         LocationDistance :  $\sqrt{\Delta x^2+\Delta y^2+\Delta z^2}$
         
         DirectionDistance: $-cos(\theta)+1 \in (0,2)$　,$\theta$为两个物体方向的夹角
         BboxSizeDistance:$min\{\frac{|oldsize\_x-newsize\_x|}{max\{oldsize\_x,newsize\_x\}},\frac{|oldsize\_y-newsize\_y|}{max\{oldsize\_y,newsize\_y\}}\} \in (0,1)$ 
         PointNumDistance:$\frac{|old\_point\_num-new\_point\_num|}{max(old\_point\_num,new\_point\_num)} \in (0,1)$
         HistogramDistance: $d+=abs(old\_feature[i]-new\_feature[i]) \in(0,3)$
         CentroidShiftDistance: $\sqrt{\Delta x^2+\Delta y^2}$
         BboxIouDistance:  $dist = (1-iou)*match\_threshold$  其中match_threshold = 4.0       
       
      - 然后进行关联,前景关联采用`<MultiHmBipartiteGraphMatcher>`
     
        背景关联使用`<GnnBipartiteGraphMatcher>` ,同一继承自`<BaseBipartiteGraphMatcher>`接口类
     
        ```c++
         // @brief: match interface
         // @params [in]: match params
         // @params [out]: matched pair of objects & tracks
         // @params [out]: unmatched rows
         // @params [out]: unmatched cols
         void Match(const BipartiteGraphMatcherOptions &options,
                    std::vector<NodeNodePair> *assignments,
                    std::vector<size_t> *unassigned_rows,
               std::vector<size_t> *unassigned_cols);
        ```
        实际上调用的是根据计算得到的代价矩阵，根据门控匈牙利算法:`<common::GatedHungarianMatcher>` ,其中设置参数`max_match_distance=4`,`bound_value=100`
        

   - 对于已关联的object和track,执行下列函数，将object添加到track的缓冲区`cached_objects`中。
   
     ```C++
     // @brief: 将已经与track关联的object添加到缓冲区
     // @param[in]: obj与track关联的新检测物体
    void MlfTrackData::PushTrackedObjectToCache(TrackedObjectPtr obj) {...}
     ```

   - 对于未被关联的objects，需要创建新的tracks,更新`MlfTrackData`中与跟踪相关的状态
   
     ```c++
     // @brief: initialize new track data and push new object to cache
     // @params [in/out]: new track data
     // @params [in/out]: new object
     void MlfTracker::InitializeTrack(MlfTrackDataPtr new_track_data,
                                   TrackedObjectPtr new_object) {...}
     ```
   
     创建新的track的过程：从`MlfTrackDataPool`对象池中创建`MlfTrackDataPtr`实例`track_data`,然后通过
     `<MlfTracker>`类中的`InitializeTrack`对创建的`track_data`结合提供的`object`信息，进行相关跟踪属性的赋值，然后将得到的`track_data`加入到已存在的跟踪列表`std::vector<MlfTrackDataPtr>`中。

5. state filter in tracker if is main sensor (default : Velodyne64)

   ```c++
   // @brief: filter tracks
   // @params [in]: tracks for filter
   // @params [in]: frame timestamp
   void MlfEngine::TrackStateFilter(const std::vector<MlfTrackDataPtr>& tracks,
                                    double frame_timestamp) {...}
   ```

   输入是上一步得到的`std::vector<MlfTrackDataPrt>` 

   - 从CachedObjects(latest_tracked_time和object的pair)获取持续时间超过阈值的,同时删除小于时间阈值的object.

     ```c++
     // @brief: get/clean track objects from cached based trackdata
     // @param[in/out]: objects
     // @note :每一个跟踪物体都有一个对应的trackdata
     void MlfTrackData::GetAndCleanCachedObjectsInTimeInterval(
         std::vector<TrackedObjectPtr>* objects) {...}
     ```

     这里有两个时间`latest_visible_time_`(最近一次可见的时间)及`latest_cached_time_`(上一次被加入到缓冲区的时间)。
     `catched_object->timestamp<=latest_visible_time_`则删除，
     `catched_object->timestamp<=latest_cached_time_`则将该cached_object添加到objects中

   - 根据从`catched_objects`中获得的objects,更新TrackData

     ```c++
     // @brief: update track data with object
     // @params [in/out]: history track data
     // @params [in/out]: new object
     void MlfTracker::UpdateTrackDataWithObject(MlfTrackDataPtr track_data,
                                                TrackedObjectPtr new_object) {...}
     ```

     1. state fitler and store belief in new_object ,(filters contain `MlfShapeFilter` `MlfMotionFilter`)

        ```c++
        // @brief: updating shape filter with object
        // @params [in]: options for updating
        // @params [in]: track data, not include new object
        // @params [in/out]: new object for updating
        void MlfShapeFilter::UpdateWithObject(const MlfFilterOptions& options,
                                              const MlfTrackDataConstPtr& track_data,
                                              TrackedObjectPtr new_object) {...}
        ```

        此处未详细看，基本处理包括计算object polygon;方向的滑动平均过滤;更新new object的`ouput_center`,
        `output_direction`,`output_size`

        ```c++
        // @brief: updating motion filter with object
        // @params [in]: options for updating
        // @params [in]: track data, not include new object
        // @params [in/out]: new object for updating
        void MlfMotionFilter::UpdateWithObject(const MlfFilterOptions& options,
                                               const MlfTrackDataConstPtr& track_data,
                                               TrackedObjectPtr new_object) {...}
        ```

        此处亦未详细看，过滤方法采用KalmanFilter,此处的基本处理过程包括了:
        若object的track_data->age为0,则初始化状态，若该object为背景则不进行处理，否则
        　首先计算相关的测量速度值(anchor_point_velocity,bbox_center_velocity,bbox_corner_velocity),根据运动一致性选取速度`selected_measured_velocity` 同时根据object点的数量差异和关联分数评估测量的质量
        `update_quality`，代码如下：

        ```c++
        1. motion_measurer_->ComputeMotionMeasurment(track_data, new_object);
        ```

        　然后使用自适应鲁棒卡尔曼滤波对track的状态估计，并剔除异常数据的影响，相较于传统的卡尔曼滤波，此处的修改包括：

        > - 在一系列的重复观测中选择速度测量，即滤波算法的输入包括了锚点以为，边界框中心偏移，边界框角点偏移等。卡尔曼滤波更新的观测值为速度，每次观测三个速度值:锚点移位速度，边界框中心偏移速度，边界框角点位移速度，从这三个速度中，根据运动一致性约束，选取和之前观测速度偏差最小的速度作为最终的观测值，并根据最近３次的速度观测值，计算加速度的观测值。
        >
        > - 在过滤中使用故障阈值`breakdown_threshold_`，当更新的增益过大时，用于克服增益的过度估计，其中速度的故障阈值是动态计算的，与速度误差协方差矩阵有关，而加速度的故障阈值是一个定值，默认为２.
        > - 更新关联质量，原始的卡尔曼滤波在更新状态时不区分测量的质量，此处使用两种策略来计算更新关联质量，关联分数和前后两个object的点云数量变化,之后取得分小的结果控制滤波器噪声。

        ```c++
        2. KalmanFilterUpdateWithPartialObservation(track_data, latest_object,
                                                   new_object);
        ```

        　然后进行convergence估计

        ```c++
        3. ConvergenceEstimationAndBoostUp(track_data, latest_object, new_object);
        ```

        ​    然后object中的belirf属性复制到output属性

        ```c++
        4. BeliefToOutput(new_object);
        ```

        ​	然后进行后处理：

        ```c++
        5. motion_refiner_->Refine(track_data, new_object)
        ```

        ​	最后进行在线协方差估计:

        ```c++
        6. OnlineCovarianceEstimation(track_data, new_object);
        ```

     2.  push new_object to track_data 更新object成为新的track:

        ```c++
        void MlfTrackData::PushTrackedObjectToTrack(TrackedObjectPtr obj) {...}
        ```

6. track to object if is main sensor ,因为objects可能来自于多个不同的Lidar传感器，此处是在主传感器上进行汇总。

   ```c++
   // @brief: collect track results and store in frame tracked objects
   // @params [in/out]: lidar frame
   void MlfEngine::CollectTrackedResult(LidarFrame* frame) {...}
   ```

   综合前面得到的`foreground_track_data_`以及`background_track_data_`通过函数`MlfTrackData::ToObject`填充`<LidarFrame>`数据结构

7. remove stale data 删除过期数据(latest_visible_time+reserved_invisible_time >= timestamp)

   ```c++
   // @brief: remove stale track data for memory management
   // @params: name
   // @params: timestamp
   // @params [in/out]: tracks to be cleaned
   void MlfEngine::RemoveStaleTrackData(const std::string& name, double timestamp,
                                        std::vector<MlfTrackDataPtr>* tracks) {...}
   ```



跟踪主要流程总结如下：

- 构造跟踪对象并将其转换为世界坐标
- 预测现有跟踪列表的状态，并进行匹配
- 在更新后的跟踪列表中更新运动窗台，并收集跟踪结果

---

### Classify

主入口函数：

```c++
// @brief: classify object list, and fill type in object.
// @param [in]: options
// @param [in/out]: object list
bool FusedClassifier::Classify(const ClassifierOptions& options,
                               LidarFrame* frame) {...}
```



首先将objects加入到序列`<ObjectSequence>`中,序列是track_id与对应object的map

```c++
// @brief: add tracked objects to sequence,and remove stale tracks
// @param [in]: objectptr ,timestamp
// @return: true or false

bool ObjectSequence::AddTrackedFrameObjects(
    const std::vector<ObjectPtr>& objects, TimeStampKey timestamp) {...}
```

物体通用类型包括：

```c++

// @brief general object type
enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};
```

如果object是背景，则将物体类型置为`ObjectType::UNKNOWN_UNMOVABLE`

否则，首先获取特定track_id在特定时间段内对应的object列表，即获取一个被跟踪物体的序列：

> (Time_1,TrackedObject_1),
> (Time_1,TrackedObject_2),
> ....
> (Time_n,TrackedObject_n),

```c++
// @brief: 获取特定track_id在到当前时刻的往前window_time时间序列中对应的object组成的列表
// @param[in]: track_id , window_time
// @param[in/out]: track (在window_time时间段内的track_id的object)
bool ObjectSequence::GetTrackInTemporalWindow(TrackIdKey track_id,
                                              TrackedObjects* track,
                                              TimeStampKey window_time) {...}
```

然后进行类型融合：

```c++
bool CCRFSequenceTypeFusion::TypeFusion(const TypeFusionOption& option,
                                        TrackedObjects* tracked_objects) {...}
```

- ```c++
  // @brief: Enter fuse with conditional probability inference
  // @param[in]: TrackedObject with timestamp
  bool CCRFSequenceTypeFusion::FuseWithConditionalProbabilityInference(
    TrackedObjects* tracked_objects) {...}
  ```
  
  - 对于跟踪序列中的每一个物体，执行`one_shot_fuser_.FuseOneShotTypeProbs`
  
    > 整个过程共分两步：
    >
    > - 状态平滑
    > - Viterbi算法推断状态
  
    
  
    ```c++
    // @brief: fuse on shot prons in sequence
    // @param[in]: ObjectPtr(the object in sequence)
    // @param[out]: log prob(fused oneshot probs type:a vector [valid_type_num,1])
    
    bool CCRFOneShotTypeFusion::FuseOneShotTypeProbs(const ObjectPtr& object,
                                                     Vectord* log_prob) {...}
    ```
  
    `smooth_matrices_:`
  
    ```c++
    DecisionForestClassifier
    0.7751 0.0298 0.0639 0.1312 // -> sum = 1.0
    0.2510 0.6802 0.0615 0.0073
    0.1904 0.0628 0.6314 0.1155
    0.1054 0.0003 0.0038 0.8905
    
    CNNSegmentation
    0.9095 0.0238 0.0190 0.0476
    0.3673 0.5672 0.0642 0.0014
    0.1314 0.0078 0.7627 0.0980
    0.3383 0.0017 0.0091 0.6508
    
    //对应四种有效类型：UNKNOWN,PEDESTRIAN,BICYCLE,VEHICLE
    ```
  
    `confidence_smooth_matrix_:`
  
    ```c++
    Confidence
    1.00 0.00 0.00 0.00
    0.40 0.60 0.00 0.00
    0.40 0.00 0.60 0.00
    0.50 0.00 0.00 0.50
    ```
  
    计算不考虑整个序列的单独各个object的概率single_prob:
    $$
    p(c|x) = p(c|x,o)p(o|x)+p(c|x,\widetilde{o})p(\widetilde{o}|x)
    $$
    程序表示为, 输入为x条件下类别的概率 = 前景的概率 * 类别概率+（背景的概率）* 类别的置信度平滑矩阵 * 类别概率
  
    ```c++
    single_prob = conf * single_prob +
                    (1.0 - conf) * confidence_smooth_matrix_ * single_prob;
    ```
  
    转换为对数概率 $log\_prob=log(single\_prob)$ ,获得该object属于各个类别的对数概率。由此可得到该
    TrackedObjects对应序列的每一个object的4种有效类型概率。
  
  - 通过维特比(Viterbi)算法推断状态：
  
    > 维特比算法前提是状态链是马尔可夫链，**下一时刻的状态仅仅取决于当前时刻的状态**
    > 假设隐状态数量为m,观测状态的数量为n,隐状态分别为$s_1,s_2,...,s_m$,可观测状态分别为
    > `o_1,o_2,...,o_n`,则有：
    > 状态转移矩阵$P(m\times m):P[i,j]$ 代表状态i到状态j转移的概率,$\sum_{j=0}^mP[i,j]=1$ ,发射概率矩阵
    > $R(m\times n):R[i,j]$代表隐状态i能被观测到为j的概率:$\sum_{j=0}^nP[i,j]=1$ 
  
    
  
    添加先验知识用于抑制突然出现的物体类型：
  
    ```c++
    //UNKNOWN,PEDESTRIAN,BICYCLE,VEHICLE
        0.34     0.22     0.33    0.11 //UNKNOWN
        0.03     0.90     0.05    0.02 //PEDESTRIAN   
        0.03     0.05     0.90    0.02 //BICYCLE
        0.06     0.01     0.03    0.90 //VEHICLE
    //对应四种有效物体类型的先验知识 转移概率矩阵
    transition_matrix_alpha:1.8
    ```
  
    下面程序是根据转移概率矩阵(left->right)推断当前时刻最大概率的过程：
    认为fused_sequence_probs为隐状态，根据上一时刻的隐状态fused_sequence_probs[i-1]和状态转移矩阵求解联合状态矩阵，其中矩阵元素$fused\_sequence\_probs\_[i][j]$的求解方式为：下列代码中，fused_oneshot_probs\_是每个时刻独立的4类概率，经过平滑和log(·)处理。transition_matrix\_是状态转移矩阵P，维度为4x4，经过log(·)处理，fused_sequence_probs_为Viterbi算法推理后的修正状态(也就是真实的隐状态)。
    $$
    p(s_{i,j}, s_{i-1,k}) = p_{prv\_state=s_{i-1,k}} * P(s_j|s_k)
    $$
    
  
    ```c++
      for (std::size_t i = 1; i < length; ++i) {
        for (std::size_t right = 0; right < VALID_OBJECT_TYPE; ++right) {
          double prob = 0.0;
          double max_prob = -DBL_MAX;
          std::size_t id = 0;
          for (std::size_t left = 0; left < VALID_OBJECT_TYPE; ++left) {
            prob = fused_sequence_probs_[i - 1](left) +
                   transition_matrix_(left, right) * s_alpha_ +
                   fused_oneshot_probs_[i](right);
            if (prob > max_prob) {
              max_prob = prob; //根据转移概率推断得到当前的object (right值)最大概率
              id = left;
            }
          }
          fused_sequence_probs_[i](right) = max_prob;
          state_back_trace_[i](right) = static_cast<int>(id);
        }
      }
    ```
  
    获取类别概率后：
  
    ```c++
    RecoverFromLogProbability(&fused_sequence_probs_.back(), &object->type_probs,
                              &object->type); //将Eigen转换回std::vector得到物体类别
    ```
  
    







### 参考：

类型融合：https://blog.csdn.net/qq_41204464/article/details/102987228

跟踪：https://mp.weixin.qq.com/s?__biz=MzI1NjkxOTMyNQ==&mid=2247485879&idx=1&sn=d921ab976ebc72c502eb5df1ef7ff548&chksm=ea1e1bc5dd6992d3220bd47c5cd951f38648c060d4f9dc9bfbe5eac1a3076f920fa3e360c038&scene=21#wechat_redirect