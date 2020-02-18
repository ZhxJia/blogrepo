---
title: Apollo回调函数概览
categories:
- 无人驾驶
- 感知
tags:
- 感知
mathjax: true
---
### 一、OnReceiveImages()内部函数处理流程:
<!--more-->

1. 由FusionCameraDetectionComponent::Init()中的InitCameraListeners()创建回调函数

``` c++
void FusionCameraDetectionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> &message,
    const std::string &camera_name) {
    ...
    }
```

函数参数说明：

> apollo::drivers::Image类包含：
>
> ```protobuf
> message Image {
>   optional apollo.common.Header header = 1;
>   optional string frame_id = 2;
>   optional double measurement_time = 3;
> 
>   optional uint32 height = 4;  // image height, that is, number of rows
>   optional uint32 width = 5;   // image width, that is, number of columns
> 
>   optional string encoding = 6;
>   optional uint32 step = 7;  // Full row length in bytes
>   optional bytes data = 8;   // actual matrix data, size is (step * rows)
> }
> ```
>
> 该消息类型文件定义：`modules/drivers/proto/sensor_image.proto`

> `std::string &camera_name` 包含两类：
>
> - front_6mm
> - front_12mm

2. 进行TimeStamp的判断之后：

- 创建`apollo::perception::PerceptionObstacles` 对象*`out_message`* 

  > 类定义位于modules/perception/proto/perception_obstacle.proto
  >
  > ```protobuf
  > message PerceptionObstacles {
  >   repeated PerceptionObstacle perception_obstacle = 1;  // An array of obstacles
  >   optional common.Header header = 2;                    // Header
  >   optional common.ErrorCode error_code = 3 [default = OK];
  >   optional LaneMarkers lane_marker = 4;
  >   optional CIPVInfo cipv_info = 5;  // Closest In Path Vehicle (CIPV)
  > }
  > ```
  >
  > 

- 创建`SensorFrameMessage` 类对象*` prefused_message`*  

  > ```c++
  > class SensorFrameMessage {
  >  public:
  >   SensorFrameMessage() { type_name_ = "SensorFrameMessage"; }
  >   ~SensorFrameMessage() = default;
  >   std::string GetTypeName() { return type_name_; }
  >   SensorFrameMessage* New() const { return new SensorFrameMessage; }
  > 
  >  public:
  >   apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
  > 
  >   std::string sensor_id_;
  >   double timestamp_ = 0.0;
  >   uint32_t seq_num_ = 0;
  >   std::string type_name_;
  >   base::HdmapStructConstPtr hdmap_;
  > 
  >   base::FramePtr frame_;
  > 
  >   ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
  > };
  > ```

3. 调用函数`IntenalProc(message,camera_name,&error_code,prefused_message.get(),out_message.get())` 

> ```c++
> int FusionCameraDetectionComponent::InternalProc(
> const std::shared_ptr<apollo::drivers::Image const> &in_message,
> const std::string &camera_name, apollo::common::ErrorCode *error_code,
> SensorFrameMessage *prefused_message,
> apollo::perception::PerceptionObstacles *out_message){
> ...
> }
> ```
>
> 该函数实现对接收到图像信息的处理，程序前部分实现对`prefused_message`  和 `camera_frame`相关数据与参数的赋值，获取相机到世界坐标系的仿射矩阵
>
> ```c++
> camera2world_trans_wrapper_map_[camera_name]->GetSensor2worldTrans(...);
> ```
>
> 填充图像数据,其中数据来自OnreceiveMessage接收到的消息
>
> ```c++
>   camera_frame.data_provider->FillImageData(
>       image_height_, image_width_,
>       reinterpret_cast<const uint8_t *>(in_message->data().data()),
>       in_message->encoding())
> ```
>
> 有了数据之后，科技进行算法处理了，运行`camera_obstacle_pipelin_->Perception(&camera_frame)` ,实现主要图像算法处理。
>
> - `Perception`中主要的处理函数如下
>
> > ```c++
> > bool ObstacleCameraPerception::Perception(
> >     const CameraPerceptionOptions &options, CameraFrame *frame) {
> >     ...
> >     //lane detector and postprocessor
> >         lane_detector_->Detect(lane_detetor_options, frame)
> >         ...
> >         lane_postprocessor_->Process2D(lane_postprocessor_options, frame)
> >         ...
> >         //Calibration service
> >         frame->calibration_service->Update(frame);
> >     	...
> >         lane_postprocessor_->Process3D(lane_postprocessor_options, frame)
> >         ...
> >         WriteLanelines(write_out_lane_file_, lane_file_path, frame->lane_objects)
> >     ...
> >     // Obstacle prediction
> >     tracker_->Predict(tracker_options, frame)      
> >     // detect :detector 根据provider名称建立
> >     detector->Detect(detector_options, frame)
> >     WriteDetections(...)//write all detections results as kitti format
> >     //extarctor
> >     extractor_->Extract(extractor_options, frame)
> >     //tracker Associate2D
> >     tracker_->Associate2D(tracker_options, frame)
> >     //Transform
> >     transformer_->Transform(transformer_options, frame)
> >     //Obstacle postprocessor
> >     obstacle_postprocessor_->Process(obstacle_postprocessor_options,
> >                                         frame)
> >     //tracker Associate3D
> >     tracker_->Associate3D(tracker_options, frame)
> >     //track
> >     tracker_->Track(tracker_options, frame)
> >     //save tracked detections results as kitti format
> >     WriteDetections(tracked_detection_out_dir,frame->tracked_objectes);
> >     //填充多边形并设置锚点
> >     FillObjectPloygonFromBBox3D(tracked_object)
> > }
> > ```
>

4. make protobuf message

   ```c++
   MakeProtobufMsg(msg_timestamp, seq_num_, camera_frame.tracked_objects,
                         camera_frame.lane_objects, *error_code,
                         out_message) 
   ```

   > ```c++
   > int FusionCameraDetectionComponent::MakeProtobufMsg(
   >     double msg_timestamp, int seq_num,
   >     const std::vector<base::ObjectPtr> &objects,
   >     const std::vector<base::LaneLine> &lane_objects,
   >     const apollo::common::ErrorCode error_code,
   >     apollo::perception::PerceptionObstacles *obstacles) {
   >     ...
   > }
   > ```
   >
   > 

5. Determine CIPV



### 二、detector and tracker

​	目前发表的相关检测跟踪的算法(ResNet,YOLO等)主要面向广泛的计算机视觉的应用，与自动驾驶领域中的检测和跟踪还是存在一定的区别的，由于汽车行驶在结构化、规则化的道路上，面向的场景更为具体，有很多的几何约束可以用于检测；其次，自动驾驶中的检测模型需要输出的信息更多，包括了障碍物的尺寸、朝向、速度等信息，然而如果以上任务都分别由专用的模块进行处理，则对系统负担较大，处理流程太长，因此还需要做**多任务学习**和网络结构的适配。

​	一个完整的系统除了深度学习模型，还需要做一些后处理，后处理模块针对下游模块，对后续的影响比较直接。在视觉感知中，后处理主要分为三个部分：

​	第一是2D-3D的几何计算，2D到3D的转换需要考虑的因素包括:

> - 相机pose 的影响
> - 接地点
> - 稳定性

​	第二是时序信息计算，主要针对跟踪处理，需要注意以下几点：

> - 对相机帧率和延时有要求，要求跟踪必须是一个轻量级的模块，因为检测已经占据了大部分时间
> - 充分利用检测模型的输出信息（特征、类别等）进行跟踪
> - 可以考虑轻量级Metric Learning

​	第三是多相机的环视融合

> - 相机布局决定融合策略，要做好视野重叠



​	检测——>2D to 3D转换 ——>跟踪——>位置、速度

​	HM目标跟踪器的主要功能是跟踪分割步骤检测到的障碍物。通常，它通过将当前检测结果与现有跟踪列表相关联的方式，形成和更新跟踪列表。如果原来的目标都不在出现则删除旧跟踪列表，在确认新的检测结果之后会生成新的跟踪列表。

​	关联之后，将会估计更新后的跟踪列表的运动状态。HM目标跟踪器使用Hungarian算法（匈牙利算法）对检测和跟踪(detection-to-track)进行关联，使用Robust Kalman Filter（鲁棒卡尔曼滤波）进行运动估计。

- 检测到跟踪关联

  将检测与现有的跟踪列表进行关联时，Apollo构建了一个二分图并使用Hungarian算法对检测和跟踪(detection-to-track)进行关联，使用`Robust Kalman Filter` (鲁棒卡尔曼滤波器)进行运动估计。

- 计算关联距离矩阵

  首先建立一个关联距离矩阵。一个给定的检测和跟踪之间的距离可以通过一系列关联属性进行计算，这些关联属性包括运动一致性和外观一致性。`HM` 跟踪器中距离计算中使用的一些属性如下所示：

  ​	

  | 关联属性名称       | 评估一致性的说明 |
  | ------------------ | ---------------- |
  | location_distance  | 运动             |
  | direction_distance | 运动             |
  | bbox_size_distance | 外观             |
  | point_num_distance | 外观             |
  | histogram_distance | 外观             |

  

**(1)tracker->Predict()**

```c++
bool OMTObstacleTracker::Predict(const ObstacleTrackerOptions &options,
                                 CameraFrame *frame) {
  for (auto &target : targets_) {
    target.Predict(frame);
    auto obj = target.latest_object;
    frame->proposed_objects.push_back(obj->object);
  }
  return true;
}
```

对跟踪列表中的每一个target通过kalman相关模型进行预测，获得新图像中的候选障碍物目标添加到`frame->proposed_objects`中

```c++
void Target::Predict(CameraFrame *frame) {
  auto delta_t =
      static_cast<float>(frame->timestamp - latest_object->timestamp);
  if (delta_t < 0) {
    return;
  }
  image_center.Predict(delta_t);
  float acc_variance = target_param_.world_center().process_variance();
  float delta_t_2 = delta_t * delta_t;
  float pos_variance = 0.25f * acc_variance * delta_t_2 * delta_t_2;
  float vel_variance = acc_variance * delta_t_2;
  world_center.process_noise_(0, 0) = pos_variance;
  world_center.process_noise_(1, 1) = pos_variance;
  world_center.process_noise_(2, 2) = vel_variance;
  world_center.process_noise_(3, 3) = vel_variance;
  world_center.Predict(delta_t);

  // const position kalman predict
  world_center_const.process_noise_.setIdentity();
  world_center_const.process_noise_(0, 0) = vel_variance * delta_t_2;
  world_center_const.process_noise_(1, 1) =
      world_center_const.process_noise_(0, 0);
  world_center_const.Predict(delta_t);
}
```

**(2) detector->Detect**
	根据图像信息(frame)检测障碍物,接口定义如下：

```c++
  // @brief: detect obstacle from image.
  // @param [in]: options
  // @param [in/out]: frame
  // obstacle type and 2D bbox should be filled, required,
  // 3D information of obstacle can be filled, optional.
  virtual bool Detect(const ObstacleDetectorOptions &options,
                      CameraFrame *frame) = 0;
```

其中主要进行前向推理过程(inference),apollo支持的推理框架由`Inference`这个协议类确定，并通过"inference_factory"进行对象创建，目前支持：`CaffeNet`、`RTNet`、`RTNetInt8`、`PaddleNet`这几种类型的模型结构。

> RTNet是TensorRT框架生成的模型，TensorRT是高性能的深度学习**推理**优化器，将跨平台的"Tensorflow,caffe,MxNet,Pytorch"等深度学习框架训练好的模型解析后进行统一的部署。

主要检测部分的函数：

```c++
get_objects_gpu(yolo_blobs_, stream_, types_, nms_, yolo_param_.model_param(),
                  light_vis_conf_threshold_, light_swt_conf_threshold_,
                  overlapped_.get(), idx_sm_.get(), &(frame->detected_objects));
```

其实现位于`region_output.cu`中,最终检测检测结果添加到`frame->detected_objects`,各输入参数含义如下。

`yolo_blobs_`:为yolo前向推断模型结构，blob为一个封装了`SyncedMemory`的包装器，作为图像，特征图的基本计算单元
`stream_`:由cudaStream_t定义cuda中的流，可以实现一个设备上同时运行多个核函数。
`types_`:为检测的目标类型;  `nms_`: 为nms的相关参数 ;  `yolo_param_.model_param()`:模型的相关参数(一些阈值)
`light_vis_conf_threshold_`和`light_swt_conf_threshold_`: vis指visible,swt指swtich 值得是车灯检测的相关阈值

> cuda-c (后缀名`.cu`)方便我们利用GPU并行处理来加速程序的运行速度





