---

title: Apollo中fusion_camera_deteciton组件消息类型
categories:
- autonomous
- apollo
tags:
- apollo
mathjax: true
---

本文主要涉及Apollo感知部分Apollo的消息接受和发送，不涉及算法程序

<!--more-->

整个`fusion_camera_detection_component`组件的配置文件:

`moudles/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`

## 接收（reader）

**MotionService**

```c++
const std::string &channel_name_local = "/apollo/perception/motion_service";
std::function<void(const MotionServiceMsgType &)> motion_service_callback =
    std::bind(&FusionCameraDetectionComponent::OnMotionService, this,
              std::placeholders::_1);
auto motion_service_reader =
    node_->CreateReader(channel_name_local, motion_service_callback);
```

对应的回调函数:

```c++
// On receiving motion service input, convert it to motion_buff_
void FusionCameraDetectionComponent::OnMotionService(
    const MotionServiceMsgType &message) {
    ...
    motion_buffer_->push_back(vehicledata);
    }
```

消息格式定义：`modules/perception/proto/motion_service.proto`

```protobuf
message MotionType {
  optional float m00 = 1;
  optional float m01 = 2;
  optional float m02 = 3;
  optional float m03 = 4;
  optional float m10 = 5;
  optional float m11 = 6;
  optional float m12 = 7;
  optional float m13 = 8;
  optional float m20 = 9;
  optional float m21 = 10;
  optional float m22 = 11;
  optional float m23 = 12;
  optional float m30 = 13;
  optional float m31 = 14;
  optional float m32 = 15;
  optional float m33 = 16;
}

message VehicleStatus {
  optional float roll_rate = 1;
  optional float pitch_rate = 2;
  optional float yaw_rate = 3;
  optional float velocity = 4;
  optional float velocity_x = 5;
  optional float velocity_y = 6;
  optional float velocity_z = 7;
  optional double time_ts = 8;                          // time stamp
  optional double time_d = 9;
  optional MotionType motion = 10;
}

message Motion_Service {
  repeated VehicleStatus vehicle_status = 1;  // An array of vehicle_information stored for previous timestamps
  optional common.Header header = 2;                    // Header
}
```



**CameraListeners**:创建回调函数

```c++
 //modules/drivers/proto/sensor_image.proto 定义了Image的消息类型
typedef std::shared_ptr<apollo::drivers::Image> ImageMsgType;
std::function<void(const ImageMsgType &)> camera_callback =
    std::bind(&FusionCameraDetectionComponent::OnReceiveImage, this,
              std::placeholders::_1, camera_name);//jac!!20/1/14:回调
auto camera_reader = node_->CreateReader(channel_name, camera_callback);
```

对应的回调函数

```c++
void FusionCameraDetectionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> &message,
    const std::string &camera_name) {
    ...
        
    }
```

图像消息格式定义：

```protobuf
message Image {
  optional apollo.common.Header header = 1;
  optional string frame_id = 2;
  optional double measurement_time = 3;

  optional uint32 height = 4;  // image height, that is, number of rows
  optional uint32 width = 5;   // image width, that is, number of columns

  optional string encoding = 6;
  optional uint32 step = 7;  // Full row length in bytes
  optional bytes data = 8;   // actual matrix data, size is (step * rows)
}
```



##　发送(writer)



### out_message

```c++
std::shared_ptr<apollo::perception::PerceptionObstacles> out_message(
      new (std::nothrow) apollo::perception::PerceptionObstacles);
```

该message的消息内容定义`modules/perception/proto/perception_obstacle.proto`

 **在`InternalProc()`函数进行的相关赋值：**

```c++
MakeProtobufMsg(msg_timestamp, seq_num_, camera_frame.tracked_objects,
	camera_frame.lane_objects, *error_code,
    out_message) != cyber::SUCC)
```



### prefused_message

```c++
 std::shared_ptr<SensorFrameMessage> prefused_message(new (std::nothrow)
                                                           SensorFrameMessage);
```

该message的消息内容定义:

```c++
class SensorFrameMessage {
 public:
  SensorFrameMessage() { type_name_ = "SensorFrameMessage"; }
  ~SensorFrameMessage() = default;
  std::string GetTypeName() { return type_name_; }
  SensorFrameMessage* New() const { return new SensorFrameMessage; }

 public:
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;

  std::string sensor_id_;
  double timestamp_ = 0.0;
  uint32_t seq_num_ = 0;
  std::string type_name_;
  base::HdmapStructConstPtr hdmap_;

  base::FramePtr frame_;

  ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
};
```

**在`InternalProc()`函数进行的相关赋值：**

```c++
int FusionCameraDetectionComponent::InternalProc(
  const std::shared_ptr<apollo::drivers::Image const> &in_message,
  const std::string &camera_name, apollo::common::ErrorCode *error_code,
  SensorFrameMessage *prefused_message,
  apollo::perception::PerceptionObstacles *out_message) {
  ...  
  prefused_message->timestamp_ = msg_timestamp;
  prefused_message->seq_num_ = seq_num_;
  //单目相机检测阶段
  prefused_message->process_stage_ = ProcessStage::MONOCULAR_CAMERA_DETECTION; 
  prefused_message->sensor_id_ = camera_name;
  prefused_message->frame_ = base::FramePool::Instance().Get();
  prefused_message->frame_->sensor_info = sensor_info_map_[camera_name];
  prefused_message->frame_->timestamp = msg_timestamp
  ...
  prefused_message->frame_->sensor2world_pose = camera2world_trans;
  prefused_message->frame_->objects = camera_frame.tracked_objects; 
    
}
```

### **最终发送**

```c++
  writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_obstacles_channel_name_); 
  sensorframe_writer_ =
      node_->CreateWriter<SensorFrameMessage>(prefused_channel_name_);
  camera_viz_writer_ = node_->CreateWriter<CameraPerceptionVizMessage>(
      camera_perception_viz_message_channel_name_);
  camera_debug_writer_ =
      node_->CreateWriter<apollo::perception::camera::CameraDebug>(
          camera_debug_channel_name_);
```

消息格式的定义基本可以在`modules/perception/proto/..`下找到

```c++
if (output_camera_debug_msg_){
    ...
	camera_debug_writer_->Write(camera_debug_msg);
}
```

```c++

if (enable_visualization_) {
	...
	bool send_viz_ret = camera_viz_writer_->Write(viz_msg);
}
```

```c++
if (output_final_obstacles_) {
    writer_->Write(out_message);
}
```

```c++
bool send_sensorframe_ret = sensorframe_writer_->Write(prefused_message);
```

