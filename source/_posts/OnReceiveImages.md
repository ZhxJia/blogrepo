---
title: Apollo回调函数概览
categories:
- 无人驾驶
- 感知
tags:
- 感知
mathjax: true
---
OnReceiveImages()内部函数处理流程:
<!--more-->

[TOC]

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
> 该函数实现对接收到图像信息的处理，程序前部分实现对`prefused_message`  和 `camera_frame`相关数据参数的赋值,然后运行`camera_obstacle_pipelin_->Perception(&camera_frame)` 
>
> 主要的处理函数如下
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
