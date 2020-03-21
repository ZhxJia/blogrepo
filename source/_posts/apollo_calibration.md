---
title: Apollo中的标定
tags:
- 感知
- 标定
- 视觉
categories:
- apollo
- perception
mathjax: True
---

Apollo中的在线标定：用于获取实时外参(针对由于路面颠簸等情况造成相机位姿改变的校正)，由于在线标定算法用到车道线信息，因此此处将车道线检测的相关算法一并说明。

<!-- more -->

## LaneDetector

算法接口文件:`modules/perception/camera/lib/interface/base_lane_detector.h`
该算法实现从图像中检测车道线。

```c++
  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  virtual bool Detect(const LaneDetectorOptions& options,
                      CameraFrame* frame) = 0;
```

### 1.初始化

**配置文件信息**：

```c++
lane_param {
  lane_detector_param {
    plugin_param {
      name : "DarkSCNNLaneDetector"
      root_dir : "/apollo/modules/perception/production/data/perception/camera/models/lane_detector/"
      config_file : "config_darkSCNN.pt"
    }
    camera_name : "front_6mm"
  }
  lane_postprocessor_param {
    name : "DarkSCNNLanePostprocessor"
    root_dir : "/apollo/modules/perception/production/data/perception/camera/models/lane_postprocessor/darkSCNN/"
    config_file : "config.pt"
  }
}
```

**初始化程序**：
`ObstacleCameraPerception::Init()`中执行`InitLane()`进行初始化

```c++
// @brief: 车道线检测初始化
// @param [in]: work_root:当前工作路径，通过`GetCyberWorkRoot()`可以获取
// @param [in]: model:相机传感器模型，通过SensorManage类加载`perception_gflags.cc`中的对应文件获取
// @param [in]: perception_param_:上面参数配置文件的相关信息
void InitLane(const std::string &work_root,
                const base::BaseCameraModelPtr model,
                const app::PerceptionParam &perception_param);
```

车道线的初始化包括两大部分，一是检测器detector初始化，二是后处理其postprocessor初始化。

**检测器detector初始化**
由于车道线检测这里用到了网络模型，因此detector初始化过程主要加载了模型的配置文件，以及具体的功能实现类
`DarkSCNNLaneDetector`,然后同障碍物检测的detector相似 进行`DarkSCNNLaneDetector`的初始化:
需要注意的是，`LaneDetector`的实现在`camera/lib/lane/detector/darkSCNN`,
而检测器参数配置则位于`production/data/perception/camera/models/lane_detector`下。

> model_name -> darkSCNN
> proto_flie -> deploy.prototxt
> weight_file -> deploy.caffemodel

网络的输入输出节点名称

> input_blob : "data"
> vpt_blob : "fc_out" //消失点检测
> seg_blob : "softmax" //分割输出

根据上述参数创建网络的推断模型`CreateInferenceByName`，此处`cnnadapter_lane_`的推断模型为`caffe`,然后对此推断模型进行初始化，之前的创建过程并未将参数加载到`caffe`模型中，因此`cnnadapter_lane_->Init`将网络模型结构和网络参数导入到Caffe中。

**后处理器postprocessor初始化**

```c++
lane_map_width: 640
lane_map_height: 480
roi_height: 768
roi_start: 312
roi_width: 1920
```

后处理器实现类`DarkSCNNLanePostprocessor`,该功能类的初始化加载上面所示参数，同时也加载了`detector`的部分参数(图像裁剪相关)

### 2. 算法实现

算法实现的相关过程位于`obstacle_camera_perception.cc`中的`Perception`函数中,并注意车道线检测算法只工作于
`front_6mm`相机上,算法流程如下,然后分模块进行说明。

```c++
//Detect -> Process2D(postprocessor) -> Update(Calibration) -> Process3D(postprocessor)
```

#### 2.1 Detect

- 准备图像数据，并将图像数据拷贝到input_blob中
  首先对原始图像(`1920*1080`)进行裁剪，得到ROI图像，(`1920*1080`)->(`1920*768`)

  由于`DarkSCNN`网络结构输出为`640*480`,所以还需要`resize,`通过GPU实现图像大小的转换。

  ```c++
  // @brief: 通过GPU对每个像素并行处理进行resize后的图像
  // @param [in]: image_src_:(dataprovider提供的原始图像)
  // @param [in/out]: input_blob(将数据拷贝到input_blob中) 
  // @param [in]: image_mean(将图像减去此均值)
  // @note : channel_axis=fasle(input_blob:nchw src:nhwc) channel_axis=true(dst=src=nhwc)
  inference::ResizeGPU(
        image_src_, input_blob, static_cast<int>(crop_width_), 0,
        static_cast<float>(image_mean_[0]), static_cast<float>(image_mean_[1]),
        static_cast<float>(image_mean_[2]), false, static_cast<float>(1.0));
  ```
```
  
  通过上述程序将图像数据拷贝到了网络模型的输入input_blob中，接下来进行前向推断，获取结果。

- 前向推断`cnnadapter_lane_->Infer()`,获取网络模型输出。

- 将网络输出seg_blob数据转换到图像矩阵Mat中,其中每一个通道c代表一类车道线,然后遍历图片中每一个像素位置的类别(通道c)大于阈值`0.95`则将打上对应的类别标签，最后将处理好的像素为对应类别标签的`Mat`转换为`blob`类型

  `frame->lane_detected_blob = lane_blob_;`将检测数据保存到`frame`帧结构中即完成了车道线检测的数据获取任务。

 在`darkSCNN`网络中，除了输出车道线的语义分割像素位置，还同时设计了消失点检测的子网络，通过最后的全连接层输出消失点：
网络输出的消失点坐标是相对于输入图像中心的偏移，将消失点恢复到网络的输入图像中，然后再根据roi裁剪恢复到原始图像中，同样将消失点坐标保存到`frame`数据结构中，` frame->pred_vpt = v_point`

通过对网络输出数据的初步处理，将检测信息保存到`frame`中，原始检测数据的获取部分就完成了，接下来进行数据的后处理。

#### 2.2 Postprocess 2D

​```c++
  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  bool Process2D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;
```

该部分主要通过检测器得到的语义分割信息提取车道线，车道线的数据结构为`modules/perception/base/lane_struct.h`

```c++
struct LaneLine {
  LaneLineType type;
  // @brief laneline position -> 13
  LaneLinePositionType pos_type;
  // @brief image coordinate system
  LaneLineCubicCurve curve_car_coord;
  // @brief camera coordinate system
  LaneLineCubicCurve curve_camera_coord;
  // @brief image coordinate system
  LaneLineCubicCurve curve_image_coord;
  // @brief curve image point set
  std::vector<Point2DF> curve_image_point_set;
  // @brief curve camera point set
  std::vector<Point3DF> curve_camera_point_set;
  // @brief curve car coord point set, only on XY plane
  std::vector<Point2DF> curve_car_coord_point_set;
  // @brief image end point set
  std::vector<EndPoints> image_end_point_set;
  // @brief track id
  int track_id = -1;
  // @brief confidence for lane line
  float confidence = 1.0f;

  LaneLineUseType use_type;
};
```

车道线检测过程可以分为四个阶段：

- 检测车道线边界，以车辆为中心，左侧的车道线提取右边界，右侧的车道线提取左边界。
  
- 根据提取的边界线点通过`ransac`采样去除离群点。
  首先确定车道线拟合方程$y=ax^3+bx^2+cx+d$,因此通过`ransac`确定车道线方程程序中至少需要8个点。
  程序中貌似仅仅拟合了二阶(b,c,d)？

  ```c++
  // @brief: 拟合边界点,去除离群值
  // @param [in]: pos_vec(地平面下的边界点)
  // @param [in/out]: selected_points(去除离群值之后的边界点)
  // @param [in]: coeff(多项式系数,a,b,c,d)
  // @param [in]: N(所需的最少点数minNumPoints=8)
  bool RansacFitting(const std::vector<Eigen::Matrix<Dtype, 2, 1>>& pos_vec,
                     std::vector<Eigen::Matrix<Dtype, 2, 1>>* selected_points,
                     Eigen::Matrix<Dtype, 4, 1>* coeff, const int max_iters = 100,
                     const int N = 5, Dtype inlier_thres = 0.1) 
  ```

- [1]对某些特殊情形下的车道线类型进行置换（比如车辆压在边界线上，或者已经到达左右边界），然后开始向`LaneLine`数据类型中填充数据。
  [2]填充对应13种车道线类型：

  ```c++
  std::map<base::LaneLinePositionType, int> spatialLUTind = {
      {base::LaneLinePositionType::UNKNOWN, 0},
      {base::LaneLinePositionType::FOURTH_LEFT, 1},
      {base::LaneLinePositionType::THIRD_LEFT, 2},
      {base::LaneLinePositionType::ADJACENT_LEFT, 3},
      {base::LaneLinePositionType::EGO_LEFT, 4},
      {base::LaneLinePositionType::EGO_CENTER, 5},
      {base::LaneLinePositionType::EGO_RIGHT, 6},
      {base::LaneLinePositionType::ADJACENT_RIGHT, 7},
      {base::LaneLinePositionType::THIRD_RIGHT, 8},
      {base::LaneLinePositionType::FOURTH_RIGHT, 9},
      {base::LaneLinePositionType::OTHER, 10},
      {base::LaneLinePositionType::CURB_LEFT, 11},
      {base::LaneLinePositionType::CURB_RIGHT, 12}};
  ```

  [3]根据拟合的车道线曲线，x=3时对应的y值判断车道线是否无效(即车道线的排列顺序对应y值应有的顺序)。
  [4]将车道线参数值添加到车道线数据结构`LaneLine`中，并将此数据添加到`Frame`中
  	`frame->lane_objects.push_back(cur_object);`
  [5]针对车辆位于左边或者右边的车道线上，修改对应一边的车道线的类型标签。

#### 2.3 



----

## Calibration_service

算法接口文件`modules/perception/camera/lib/interface/base_calibration_service.h`

```c++
// @brief query camera to world pose with refinement if any
  virtual bool QueryCameraToWorldPose(Eigen::Matrix4d *pose) const {
    return false;
  }

  // @brief query depth on ground plane given pixel coordinate
  virtual bool QueryDepthOnGroundPlane(int x, int y, double *depth) const {
    return false;
  }

  // @brief query 3d point on ground plane given pixel coordinate
  virtual bool QueryPoint3dOnGroundPlane(int x, int y,
                                         Eigen::Vector3d *point3d) const {
    return false;
  }

  // @brief query ground plane in camera frame, parameterized as
  // [n^T, d] with n^T*x+d=0   ax+by+cz+d=0,法向量 n=(a,b,c)
  virtual bool QueryGroundPlaneInCameraFrame(
      Eigen::Vector4d *plane_param) const {
    return false;
  }

  // @brief query camera to ground height and pitch angle
  virtual bool QueryCameraToGroundHeightAndPitchAngle(float *height,
                                                      float *pitch) const {
    return false;
  }

  virtual float QueryCameraToGroundHeight() const { return 0.f; }

  virtual float QueryPitchAngle() const { return 0.f; }

  // @brief using calibrator to update pitch angle
  virtual void Update(CameraFrame *frame) {
    // do nothing
  }

  // @brief set camera height, pitch and project matrix
  virtual void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_master_sensor) {
    // do nothing
  }

```

`OnCalibrationService`通过车道线进行在线标定，标定参数主要包含了相机的高度和pitch角度。

同时，标定服务实际对应的`calibrator`方法的接口为：

```c++
class BaseCalibrator {
 public:
  // @brief: refine 3D location of detected obstacles.
  // @param [in]: options
  // @param [in/out]: pitch_angle
  virtual bool Calibrate(const CalibratorOptions &options,
                         float *pitch_angle) = 0;
};  // class BaseCalibrator
```

该接口根据检测得到的车道线信息以及相机到世界坐标系的位姿变换矩阵，在线标定相机`pitch`轴角度，并最终用于障碍物3D位置的改善。

### 1. 初始化

**配置文件与配置信息**
 `modules/perception/production/conf/perception/camera/obstacle.pt`

```protobuf
calibration_service_param {
  plugin_param {
      name : "OnlineCalibrationService"
      root_dir : ""
      config_file : ""
  }
  calibrator_method : "LaneLineCalibrator"
}
```

```c++
struct CalibrationServiceInitOptions : public BaseInitOptions {
  int image_width = 0;
  int image_height = 0;
  double timestamp = 0;
  std::string calibrator_working_sensor_name = "";
  std::string calibrator_method = "";
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map;
};
```


**初始化程序**：
在`ObstacleCameraPerception::Init()`中进行初始化`InitCalibrationService`
初始化过程配置在线标定的传感器为`front_6mm`,配置标定方法为:`LaneLineCalibrator`,同时加载内参矩阵和图像宽高,根据`plugin_param`中的`name`确定`calibration_service_`指向的算法实现类为`OnlineCalibrationService`。
在`OnlineCalibrationService::Init()`初始化过程中，进行标定方法`LaneLineCalibrator`的初始化,然后在初始过程中又进行了更底层实现`lane_based_calibrator`的初始化，

```c++
// @brief: 标定方法初始化，loal_options包括（图像的宽高以及相机内参f_x,f_y,c_x,c_y）
calibrator_.Init(local_options)
```

在`LaneLineCalibrator`初始过程中进行`HistogramEstimator`的初始化（**注：默认参数在构造函数中加载**）


### 2. 算法实现


#### 2.1 Online_Calibration_Service::Update

```c++
// @brief using calibrator to update pitch angle
void Update(CameraFrame *frame) override;
```

该方法通过创建`calibrator`进行相机pitch角度的校正，`calibrator`的配置信息包括:

```c++
struct CalibratorOptions {
  std::shared_ptr<std::vector<base::LaneLine>> lane_objects; //车道线检测检测信息
  std::shared_ptr<Eigen::Affine3d> camera2world_pose; //相机到世界坐标系的转换矩阵
  double *timestamp = nullptr;
};
```

然后根据车道线检测信息和位姿变换矩阵，在线标定pitch轴角度：

##### 2.1.1 LaneLineCalibrator::Calibrate
`bool updated = calibrator_->Calibrate(calibrator_options, &pitch_angle)`

```c++
  // @brief: estimate camera-ground plane pitch.
  // @param [in]: options (包括车道线检测信息，相机到世界坐标系的转换矩阵)
  // @param [in/out]: pitch_angle
  bool Calibrate(const CalibratorOptions &options, float *pitch_angle) override;
```

- `LoadEgoLaneline()`

  ```c++
  // @brief: load ego lane line from camera frame (获取车辆所在的车道线，用于标定)
  // @param [in]: lane_objects (from model fitting)
  // @param [out]: ego_lane (ego left_line,ego right_line)
  bool LoadEgoLaneline(const std::vector<base::LaneLine> &lane_objects,
                         EgoLane *ego_lane);
  ```

  **上述程序处理中存在问题，lane_objects.curve_image_coord并没有相关赋值，不知道它为何直接用了**

- `GetYawVelocityInfo()`

  ```c++
  // @brief: 获取车辆速度和yaw轴角度变化率
  // @param [in]: time_diff(两帧时间差),cam_coord_cur(当前相机在世界坐标系下位置)
  // @param [in]: cam_coord_pre(上一帧相机在世界坐标下位置),cam_coord_pre_pre()
  // @param [out]: yaw_rate(yaw角度变化率),velocity(车速)
  // @note: 车辆速度通过两帧时间差的相机位置变化确定，yaw通过世界坐标系下相机位置计算得到
  void GetYawVelocityInfo(const float &time_diff, const double cam_coord_cur[3],
                          const double cam_coord_pre[3],
                          const double cam_coord_pre_pre[3], float *yaw_rate,
                          float *velocity);
  ```

- `LaneBasedCalibrator::Process()` 计算pitch_estimation,vanishing_row

  ```c++
  // @brief: Main function. process every frame, return true if get valid
  // estimation. suppose the points in lane are already sorted.
  // @param [in]: ego_lane(车辆所在车道线的左右车道线)
  // @param [in]: velocity(相机世界坐标系下的速度)
  // @param [in]: yaw_rate(相机世界坐标系下yaw变化率)
  // @param [in]: time_diff(两帧之间的时间差) 
  // @out : pitch_estimation_, vanishing_row_
  
  bool Process(const EgoLane &lane, const float &velocity,
             const float &yaw_rate, const float &time_diff);
  ```
  
  - `IsTravelingStraight()`判断当前yaw变化率是否小于3°，即判断是否直线行驶，若不为直线行驶，则不进行处理,return false。
  
  - `GetVanishingPoint()`根据ego Lane车道线获取消失点`vp_cur`
    - `SelectTwoPointsFromLineForVanishingPoint(egoleftline/rightline)` 使用ego line的部分(或全部)线段，返回线段的起止索引。
    - `GetIntersectionFromTwoLineSegment()`使用上面的车道线线段获取消失点，即计算两条直线延长线的交点。
      ref:https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    
  - `PushVanishingPoint()`将`vp_cur`添加到`vp_buffer_`中，`vp_buffer_`中最大容量1000，最先检测到的位于最后
  
  - `PopVanishingPoint()`计算`vp_buffer`累计前行的距离，判断是否大于最低前行距离要求(20m),若满足则将末尾（最新）的消失点提出来使用->`vp_work`
  
  - `GetPitchFromVanishingPoint()`根据`vp_work`估计相机`Pitch`角度->`pitch_cur_`
    空间中一条直线的消失点是一条与该直线平行且经过相机光心的射线与像平面的交点,由于车道线与地平面水平，因此可计算pitch(与地平面夹角)
    
  - `AddPitchToHistogram()`将pitch_cur添加到hist中。
  
    - `HistogramEstimator::Push(pitch_cur_)`,首先需要注意用于标定的直方图参数的设置是在
      `CalibratorParams::Init()`中，通过将(-10,10°)区间平分为400份，然后计算输入的`pitch_cur_`所在区间的位置，并将该位置的计数值加1`hist_[index]++`。(直方图的数据分组数量为bin)
  
  - `HistogramEstimator::Process()` 当累计前行距离大于100m时，进行标定值的更新，流程可以概括为：
    ` smooth -> get peak & check mass -> shape analysis -> update estimate -> final decay`。
  
    - `Smooth(hist,nr_bins,hist_smoothed)` hist为之前添加的pitch角度计数，nr_bins为数据分组数量。原统计的直方图存在锯齿状波动，对直方图通过核<1,3,5,3,1>进行平滑以方便确定波峰和波谷。`hist_smoothed`
  
    - `GetPeakIndexAndMass(hist_smoothed,nr_bins,max_index,mass)` 获得波峰对应的索引(表示该计算得到的pitch数量最多)以及所有计数值之和`mass`,若`mass`即对应采样点的个数少于100，则缺少足够数据。
  
    - `IsGoodShape(hist,nr_bins,max_index)`，对应于初始化时的先验`hist_hat_`判断当前的直方图形状是否正常
      $$
      hist\_hat(i)=e^{-\frac{(x-200)^2}{2\sigma ^2}}\ i\in[0,400],\sigma=6.25
      $$
      <img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\apollo_calibration\hist_hat.jpg" style="zoom:30%;" />
  
      判断标准$hist(i)>hist(max\_index)\times hist\_hat(i-(max\_index-200))$ 则认为直方图形状**不符合**要求。
  
    - `GetValFromIndex(max_index)` 将对应直方图的索引值转换为角度值->`val_estimation`
    - `Decay(hist,nr_bins)` 将直方图中的技术值逐步衰减 * 0.8908987

> 总的来说，获取各个消失点所要求的最低直行距离为20m,而通过各个消失点计算的pitch角度更新标定值所需最低直行距离为100m,但是需要注意的是，一旦行驶过程中，出现不是直行的状态，则消失点缓存即刻清零，而用于更新标定值的累计前行距离只有到达可更新的阈值才会清零，不会因为没有直行而清零。

通过上述标定得到的pitch角度，校正相机`CameraStatus`
其中相机状态(camera_status)主要包含以下信息：

```c++
struct CameraStatus {
  float camera_ground_height = -1.f;
  float pitch_angle = 0.f;
  float pitch_angle_diff = 0.f;
  std::vector<double> k_matrix = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ground_plane = {0.0, 0.0, 0.0, 0.0};
};
```

```c++
	 name_camera_status_map_[master_sensor_name_].pitch_angle = pitch_angle;
      for (auto iter = name_camera_status_map_.begin();
           iter != name_camera_status_map_.end(); iter++) {
        // update pitch angle
        iter->second.pitch_angle =
            iter->second.pitch_angle_diff + iter->second.pitch_angle;
        // update ground plane param
        iter->second.ground_plane[1] = cos(iter->second.pitch_angle);
        iter->second.ground_plane[2] = -sin(iter->second.pitch_angle);
```







## supplement material

网络结构









---

## 参考文章

> [1]崔洪柳. 基于车道线检测的车载摄像机在线标定算法[D].东北大学,2015.
>
> 多传感器标定：https://gitchat.blog.csdn.net/article/details/94420885
>
> 传感器标定: https://blog.csdn.net/m0_38087936/article/details/88536345









### 单应性矩阵

由外参计算得到单应性矩阵 `visualizer`

```c++
  // Get homography from projection matrix
  // ====
  // Version 1. Direct

  // compute the homography matrix, such that H [u, v, 1]' ~ [X_l, Y_l, 1]
  Eigen::Matrix3d R = adjusted_camera2car_.block(0, 0, 3, 3);
  Eigen::Vector3d T = adjusted_camera2car_.block(0, 3, 3, 1);
  Eigen::Matrix3d H;
  Eigen::Matrix3d H_inv;

  H.block(0, 0, 3, 2) = (K_ * R.transpose()).block(0, 0, 3, 2);
  H.block(0, 2, 3, 1) = -K_ * R.transpose() * T;
  H_inv = H.inverse();
  homography_ground2image_ = H;
  homography_image2ground_ = H_inv;
```



https://blog.csdn.net/ganguowa/article/details/60765691





### 相机畸变校正　Brown模型

在摄影测量和计算机视觉中都需要对含有畸变的影像进行改正操作，常用到的模型是对透视影像改正的Brown模型





### 计算两直线交点坐标（车道线消失点）

https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect