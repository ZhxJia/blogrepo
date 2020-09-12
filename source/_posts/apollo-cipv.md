---

title: apollo中cipv检测
categories:
- autonomous
- apollo
tags:
- apollo
mathjax: true
---

Apollo中CIPV(Closest-In Path Vehicle)为当前车道最接近的车辆，对象由3D边界框表示，其从上到下视图的2D投影将对象定位在地面上，然后检查每个对象是否在当前的车道中。在当前车道的对象中，最接近的一个车辆将被选为CIPV.

<!--more-->

`CIPV`通过检测道路上的关键物体以进行纵向控制,利用的信息包括物体检测输出及自身所在车道线`(object detection,ego-lane line)`。其可用于跟车，从跟踪对象和当前车辆运动中，估计对象的轨迹。该轨迹将指导对象如何在道路上作为一组移动并且可以预测未来的轨迹。有两种跟车尾随，一种是跟随特定车辆的纯尾随，另一种是CIPV引导的尾随，当检测到无车道线时，当前车辆遵循`CIPV`的轨迹。
![](apollo-cipv/perception_visualization_apollo_3.0.png)

**apollo中感知输出的可视化，左上角是基于图像的输出，左下角显示了对象的3D边界框。右图显示了车道线和物体的三维俯视图。CIPV标有红色框，黄线表示每辆车的轨迹。**

```c++
  // We use "right handed ZYX" coordinate system for euler angles
  // adjust pitch yaw roll in camera coords
  // Remember that camera coordinate
  // (Z)----> X
  //  |
  //  |
  //  V
  //  Y

 //----------------------------------------------------------------
  //    area ID, corner ID and face ID
  //----------------------------------------------------------------
  //    8 | 1 | 2       a
  //    ---------    0-----1   ^
  //      |   |       |   |    |
  //    7 | 0 | 3    d|   |b
  //      |   |       |   |
  //    ---------    3-----2
  //    6 | 5 | 4       c
  //----------------------------------------------------------------
```



## 配置

使能`CIPV`需要在`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt` 中设置`enable_cipv:able` 

## 初始化

基本数据结构的定义位于`modules/perception/camera/app/cipv_camera.h`

`cipv`初始化位于`FuisonCameraDetectionComponent::Init()` 
初始化所需参数包括了:

- 图像与地平面之间的单应性矩阵(`homography_im2car_`)

  该矩阵由可视化`Visualizer`功能模块计算获得，计算过程可以参照`Visualizer::adjust_angles()`,这一部分的初始化`visualize_.Init_all_info_single_camera(...)`由此处执行。

- `cipv`所需要的车道线的最少点数(最短长度)`(kMinLaneLineLengthForCIPV)`

  该参数默认为2

- 车道线的平均宽度`(kAverageLaneWidthInMeter)`
  默认为3.7(米)

- 最大车辆宽度`(kMaxVehicleWidthInMeter)`

  默认为1.87米

- 平均帧率`(kAverageFrameRate)`
  默认0.05 s

- `image_based_cipv_`

  布尔值,`true`:在图像空间中检测目标是否位于车道线内，`false:`在地平面空间中检测目标是否位于车道线

- `debug_level`

  ```
   // 0: no debug message
   // 1: minimal output
   // 2: some important output
   // 3: verbose message
   // 4: visualization
   // 5: all
   // -x: specific debugging, where x is the specific number
  ```



## CIPV主体流程

运行位于`FusionCameraDetectionComponent::InternalProc()`中，在主体的相关算法(障碍物跟踪检测、车道线检测)处理完成后运行,同时该部分用到了`motion_service`的信息(`MotionBuffer`中存储的车辆状态信息`VehicleStatus`)

```c++
 //  Determine CIPV
  if (enable_cipv_) {
    CipvOptions cipv_options;
    if (motion_buffer_ != nullptr) {
      if (motion_buffer_->size() == 0) {
        AWARN << "motion_buffer_ is empty";
        cipv_options.velocity = 5.0f;
        cipv_options.yaw_rate = 0.0f;
      } else {
        cipv_options.velocity = motion_buffer_->back().velocity;
        cipv_options.yaw_rate = motion_buffer_->back().yaw_rate;
      }
      ADEBUG << "[CIPV] velocity " << cipv_options.velocity
             << ", yaw rate: " << cipv_options.yaw_rate;
      cipv_.DetermineCipv(camera_frame.lane_objects, cipv_options, world2camera,
                          &camera_frame.tracked_objects);

      // Get Drop points
      if (motion_buffer_->size() > 0) {
        cipv_.CollectDrops(motion_buffer_, world2camera,
                           &camera_frame.tracked_objects);
      } else {
        AWARN << "motion_buffer is empty";
      }
    }
  }
```

主要有两个处理函数:`DetermineCipv`,`CollectDrops` ,下面分别分析其主要实现的功能:

### DetermineCipv





### CollectDrops



