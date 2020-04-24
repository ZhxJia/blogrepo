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
   
     ```c++
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

         - 根据当前object和track预测的状态信息，分别计算上述7个特征,加权求和，函数实现位于
           `distace_collection.h`中。
           LocationDistance :$\sqrt{{\Delta x}^2+{\Delta y}^2+{\Delta z}^2}　\in (0,\infin)$ 

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

     - 设置objects中对应object的association_score,

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

     









---

### Classify

