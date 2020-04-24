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

2. add global offset to pose

3. 分割前景和背景的物体，并转换为被跟踪物体

   ```c++
     // @brief: split foreground/background objects and attach to tracked objects
     // @params [in]: objects
     // @params [in]: sensor info
     void MlfEngine::SplitAndTransformToTrackedObjects(
         const std::vector<base::ObjectPtr>& objects,
         const base::SensorInfo& sensor_info);
   ```

   - 向被跟踪列表中添加object：

     ```c++
     tracked_objects[i]->AttachObject(objects[i], sensor_to_local_pose_,
                                       global_to_local_offset_, sensor_info);
     ```

     

   - 如果object不是背景且采用直方图匹配方法,则计算物体的外观特征中的`histogram_distance`

     ```
     tracked_objects[i]->ComputeShapeFeatures();
     ```

     



---

### Classify

