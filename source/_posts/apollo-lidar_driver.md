---
title: apollo lidar驱动组件
tags:
- apollo
categories:
- autonomous
- apollo
mathjax: true
---

Lidar的驱动程序组件

<!--more-->

先搬运一下官方的velodyne驱动组件[说明文档][https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/velodyne/README_cn.md],激光雷达的驱动主要包含了四个组件：

- 数据读取打包 -->/driver  (输出数据包)
  channel: /apollo/sensor/lidar128/Scan 
  type：apollo::drivers::velodyne::VelodyneScan 

  proto: [modules/drivers/velodyne/proto/velodyne.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/velodyne/proto/velodyne.proto)

- 生成点云 --> /convert

  channel: /apollo/sensor/lidar128/PointCloud2 
  type: apollo::drivers::PointCloud
  proto:[modules/drivers/proto/pointcloud.proto][https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto]

- 点云融合 --> /fusion
  点云融合主要将多个激光雷达数据融合成一张点云

- 运动补偿 --> /compensator
  运动补偿依赖`tf`来进行坐标转换查询，因此需要和`gnss_driver`一起运行才能正常工作
  channel: /apollo/sensor/lidar128/copensator/PointCloud2 
  type : apollo::drivers::PointCloud
  proto: [modules/drivers/proto/pointcloud.proto][https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto]

启动`velodyne`驱动：
**需要先修改并确认launch文件中的参数与实际车辆相对应**

```c++
# in docker
cd /apollo && cyber_launch start modules/drivers/velodyne/launch/velodyne.launch
```

实际根据感知5.0的[说明文档][https://github.com/ApolloAuto/apollo/blob/master/modules/perception/README.md]，Lidar的输出数据包括：

- 128 线Lidar数据，(cyber channel : /apollo/sensor/velodyne128)
- 16 线Lidar数据 ，(cyber channel: /apollo/sensor/lidar_front ,lidar_rear_left,lidar_rear_right)


下面简单介绍驱动部分四个组件的处理：

## driver(velodyne_driver_component)

BUILD文件 组件编译为动态链接库

```cmake
cc_binary(
    name = "libvelodyne_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":velodyne_driver_component_lib"],
)
```

对应组件的dag文件（`modules/drivers/velodyne/dag/velodyne.dag`）中的Driver部分：

```protobuf

module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/velodyne/driver/libvelodyne_driver_component.so"

    # 128
    components {
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_128_driver"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne128_conf.pb.txt"
      }
    }
    # 16_front_up
    components {
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_16_front_up_driver"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne16_front_up_conf.pb.txt"
      }
    }
    # 16_front_center
    components {
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_16_front_center_driver"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne16_front_center_conf.pb.txt"
      }
    }
    # 16_rear_left
    components {
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_16_rear_left_driver"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne16_rear_left_conf.pb.txt"
      }
    }
    # 16_rear_right
    components {
      class_name : "VelodyneDriverComponent"
      config {
        name : "velodyne_16_rear_right_driver"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne16_rear_right_conf.pb.txt"
      }
    }
}
```

下面以128线激光雷达为例，对应的配置文件

```protobuf
frame_id: "velodyne128"
scan_channel: "/apollo/sensor/lidar128/Scan"
rpm: 600.0
model: VLS128
mode: STRONGEST
prefix_angle: 18000
firing_data_port: 2368  //velodyne socket 端口
positioning_data_port: 8308 //定位数据 socket端口
use_sensor_sync: false
max_range: 100.0
min_range: 0.9
use_gps_time: true
calibration_online: false
calibration_file: "/apollo/modules/drivers/velodyne/params/velodyne128_VLS_calibration.yaml"
organized: false
convert_channel_name: "/apollo/sensor/lidar128/PointCloud2"
use_poll_sync: true
is_main_frame: true
```

### 程序：

- VelodyneDriverComponent::Init()
  加载配置文件：`velodyne128_conf.pb.txt -> proto:/modules/drivers/velodyne/proto/config.proto` 
  创建节点：type:`<VelodyneScan>` name:`"/apollo/sensor/lidar128/Scan"` ,proto:`./velodyne/proto/velodyne.proto`
  根据配置文件`model`参数通过`VelodyneDriverFactory::CreateDriver`创建对应的驱动类`VelodyneDriver`
  激光雷达的驱动类`VelodyneDriver`包含了两个主要的接口：

  - `Init()`

    - `SocketInput::init(const int &port) ` 连接Velodyne UDP接口 

    - 创建`PollPositioningPacket`包接收线程,(以Gps的时间为基准)

      ```c++
        // raw data output topic
        positioning_thread_ =
            std::thread(&VelodyneDriver::PollPositioningPacket, this);
      ```

  - `Poll(std::shared_ptr<VelodyneScan> scan)`

  开启设备线程：

  ```c++
    device_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&VelodyneDriverComponent::device_poll, this)));
    device_thread_->detach();
  ```

- VelodyneDriverComponent::device_poll()
  在线程中，循环执行该函数拉取Lidar数据
  - `VelodyneDriver::Poll(const std::shared_ptr<VelodyneScan>& scan)`
  - `writer->Write(scan)` 
    点云数据发布通道`/apollo/sensor/lidar128/Scan`

-----

##　convert(velodyne_convert_component)

```protobuf
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/velodyne/parser/libvelodyne_convert_component.so"

    components {
      class_name : "VelodyneConvertComponent"
      config {
        name : "velodyne_convert"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne128_conf.pb.txt"
        readers {channel: "/apollo/sensor/lidar128/Scan"}
      }
    }
}
```

接受`driver`组件发布的原始Velodyne数据，在该组件中转换为点云结构数据。

### 初始化：

创建Convert类对象(根据不同的lidar模型创建`VelodyneParser`
和Writer对象(对象类型:`<PointCloud>`,发布通道：`//apollo/sensor/lidar128/PointCloud2`)
创建`PointCloud`对象池(大小为8) `CCObjectPool<PointCloud> point_cloud_pool_` 并将对象池中的
PointCloud对象中的点数设置为140000

### 处理函数：

接收`"/apollo/sensor/lidar128/Scan"`通道的原始Velodyne数据

**Convert::ConvertPacketsToPointCloud(scan_msg, point_cloud_out)**

- `Velodyne128Parser::GeneratePointcloud(scan_msg, point_cloud)` 该函数接收Scan_msg返回点云数据格式的point_cloud。

  - `Velodyne128Parser::Unpack(scan_msg->firing_pkts(i), out_msg)` 循环读取`scan_msg`所有的原始数据信息，通过该函数进行解析，每个packet有12个blocks，根据数据协议可以得到激光点的水平角度值（azimuth）,然后每个block有32个channel，结合校正文件，进一步得到每一个点的距离(distance)

    - `is_scan_valid(azimuth,distance)` 根据距离判断点是否有效：

      ```c++
        if (range < config_.min_range() || range > config_.max_range()) {
          return false;
        } //该函数目测貌似没啥用
      ```

    然后获取点的强度`intensity`, 这样由水平角度，垂直角度(在标定文件中)，距离三个信息可以换算得到三维直角坐标系中的点云坐标

    - `ComputeCoords(real_distance, corrections, azimuth_corrected, point_new);`
      三维点云坐标以ROS右手坐标系为基准建立。

    - ` Velodyne128Parser::IntensityCompensate()`

      对反射强度进行补偿，根据标定文件中的`focal_distace`,`focal_slope` 。

- `writer_->Write(point_cloud_out)` 输出`<PointCloud>`格式的数据，发布通道为：
  `/apollo/sensor/lidar128/PointCloud2`

----

## PrisecFusionComponent()

**以16线front center为基准融合rear left ,rear right的点云数据**

```protobuf
#dag组件配置
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/velodyne/fusion/libvelodyne_fusion_component.so"

    components {
      class_name : "PriSecFusionComponent"
      config {
        name : "velodyne_fusion"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne_fusion_conf.pb.txt"
        readers {channel: "/apollo/sensor/lidar16/front/center/PointCloud2"}
      }
    }
}
```

Component参数：

```protobuf
# velodyne_fusion_conf.pb.txt
max_interval_ms: 50
drop_expired_data : true
fusion_channel: "/apollo/sensor/lidar16/fusion/PointCloud2"
input_channel: [
    "/apollo/sensor/lidar16/rear/left/PointCloud2",
    "/apollo/sensor/lidar16/rear/right/PointCloud2"
]

# wait time after main channel receive msg, unit second
wait_time_s: 0.02
```

**初始化Init()**

- **Init**

  - 创建transform::Buffer实例，用于查询坐标转换信息

    ```c++
    buffer_ptr_ = apollo::transform::Buffer::Instance();
    ```

  - 创建节点writer，用于发布点云融合数据：

    ```c++
    fusion_writer_ = node_->CreateWriter<PointCloud>(conf_.fusion_channel());
    ```

  - 创建节点reader,用于监听另外两个16线雷达的数据信息。

    ```c++
      for (const auto& channel : conf_.input_channel()) {
        auto reader = node_->CreateReader<PointCloud>(channel);
        readers_.emplace_back(reader);
      }
    ```

    主通道数据通过组件Proc接收`front_center`位置的16线雷达数据，通过Proc处理。

- **Proc**

  ```c++
  bool PriSecFusionComponent::Proc(
      const std::shared_ptr<PointCloud>& point_cloud) {
    auto target = point_cloud;
    auto fusion_readers = readers_;
    auto start_time = Time::Now().ToSecond();
    while ((Time::Now().ToSecond() - start_time) < conf_.wait_time_s() &&
           fusion_readers.size() > 0) {
      for (auto itr = fusion_readers.begin(); itr != fusion_readers.end();) {
        (*itr)->Observe(); //注意此处的reader并不通过回调函数读取数据，而是通过Observe()
        if (!(*itr)->Empty()) {
          auto source = (*itr)->GetLatestObserved();
          if (conf_.drop_expired_data() && IsExpired(target, source)) {
            ++itr;
          } else {
            Fusion(target, source);
            itr = fusion_readers.erase(itr);
          }
        } else {
          ++itr;
        }
      }
      usleep(USLEEP_INTERVAL);//5000us
    }
    fusion_writer_->Write(target);
  
    return true;
  }
  ```

  主通道接收到点云数据后，在0.02s内等待接收readers的消息。注意此处readers并不由回调函数调用，而是直接通过
  `reader->Observe()`和`reader->GetLatestObserved()`获取数据。

  - `IsExpired(target,source)`

    ```c++
    // @brief: 判断通过Observe得到的两个Lidar数据是否过期
    // @param: target 主通道接收到的点云数据(front center)
    // @param: source 另外两个lidar通过observe获取得到的最新数据(rear left or rear right)
    // 根据lidar数据保重measurement_time的相隔时间不大于50ms
    bool PriSecFusionComponent::IsExpired(
        const std::shared_ptr<PointCloud>& target,
        const std::shared_ptr<PointCloud>& source)
    ```

  - `Fusion(target, source)`

    ```c++
    // @brief: 没有source 相较于target没有过期的话，则对当前帧点云数据进行融合
    // @param: target 主通道接收到的点云数据(front center)
    // @param: source 另外两个lidar通过observe获取得到的最新数据(rear left or rear right)
    bool PriSecFusionComponent::Fusion(std::shared_ptr<PointCloud> target,
                                       std::shared_ptr<PointCloud> source)
    ```

    - `QueryPoseAffine(target->header().frame_id(), source->header().frame_id(),&pose`)`

      ```c++
      // @brief: 通过transform::buffer查询source->target的坐标转换矩阵
      // @param[in]: target_frame_id, source_frame_id
      // @param[out]: pose 转换矩阵
      bool PriSecFusionComponent::QueryPoseAffine(const std::string& target_frame_id,
                                                  const std::string& source_frame_id,
                                                  Eigen::Affine3d* pose)
      ```

    - `AppendPointCloud(target, source, pose)`

      ```c++
      // @bried: 利用坐标转换将source lidar注册到 target lidar的位置中
      // @param[in/out]: point_cloud 主雷达数据(基准)
      // @param[in]: point_cloud_add 补充的雷达数据
      // @param[in]: pose: 坐标变换矩阵
      void PriSecFusionComponent::AppendPointCloud(
          std::shared_ptr<PointCloud> point_cloud,
          std::shared_ptr<PointCloud> point_cloud_add, const Eigen::Affine3d& pose) 
      ```

  - `fusion_writer_->Write(target);`将融合后的front_center lidar数据通过通道
    `/apollo/sensor/lidar16/fusion/PointCloud2`发布。

--------

## CompensatorComponent()

**对之前融合的16线fusion/Pointcloud和front up lidar, lidar128这三个通道的雷达数据进行补偿**

```c++
# 以velydone 128线激光雷达为例
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/velodyne/compensator/libvelodyne_compensator_component.so"

    components {
      class_name : "CompensatorComponent"
      config {
        name : "velodyne128_compensator"
        config_file_path : "/apollo/modules/drivers/velodyne/conf/velodyne128_fusion_compensator.pb.txt"
        readers {channel: "/apollo/sensor/lidar128/PointCloud2"}
      }
    }
}
```

Component参数：

``` protobuf
# modules/drivers/velodyne/conf/velodyne128_fusion_compensator.pb.txt
world_frame_id: "world"
transform_query_timeout: 0.02
output_channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
```

**初始化Init()**

- **Init()**

  - 创建writer输出通道,发布补偿后的lidar点云数据：

    ```c++
    writer_ = node_->CreateWriter<PointCloud>(config.output_channel());
    ```

  - 根据配置文件参数创建该组件的功能实现类`<Compensator>` ,

    ```c++
    compensator_.reset(new Compensator(config));
    ```

  - 并创建点云数据类型的对象池(大小为8)(**貌似对点云数据直接处理的都会先创建对象池**),并为创建的点云类型预先分配14000个点

    ```c++
    compensator_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
    compensator_pool_->ConstructAll();
    ```
  
- **Proc()**
  从对象池中获取点云数据结构类型point_cloud_compensated

  - `Compensator::MotionCompensation(point_cloud, point_cloud_compensated)`

    ```c++
    //@brief: 进行点云运动补偿的入口函数
    //@param[in]: msg 接收到的原始点云数据
    //@param[out]: msg_compensated 补偿后的点云数据
    bool Compensator::MotionCompensation(
        const std::shared_ptr<const PointCloud>& msg,
        std::shared_ptr<PointCloud> msg_compensated){...}
    ```

    - `GetTimestampInterval(msg, &timestamp_min, &timestamp_max)`

      ```c++
      //@brief： 由接收到的点云数据计算时间间隔
      //@param[out]: timestamp_min 该数据帧中点云中单个点的时间戳最小值
      //@param[out]: timestamp_max ... 最大值
      //@param[in]: 当前帧接收到的点云数据
      inline void Compensator::GetTimestampInterval(
          const std::shared_ptr<const PointCloud>& msg, uint64_t* timestamp_min,
          uint64_t* timestamp_max){...}
      ```

    - 设置运动补偿点云point_cloud_compensated的时间戳信息

      ```c++
        msg_compensated->mutable_header()->set_timestamp_sec(
            cyber::Time::Now().ToSecond()); //该时间戳为消息发布时刻
        msg_compensated->mutable_header()->set_frame_id(msg->header().frame_id());
        msg_compensated->mutable_header()->set_lidar_timestamp(
            msg->header().lidar_timestamp()); //该消息类型对应需要的lidar数据时刻
        msg_compensated->set_measurement_time(msg->measurement_time()); //lidar数据获取时的时间
        msg_compensated->set_height(msg->height());
        msg_compensated->set_is_dense(msg->is_dense());
      ```

    - 补偿点云，移除nan的点：

      - `QueryPoseAffineFromTF2(timestamp_min, &pose_min_time, frame_id)`  
        注： pose_max_time同样

        ```c++
        //@brief: 通过tf buffer获取timestamp时刻原始点云数据到世界坐标系的转换矩阵
        //@param[in]: timestamp:时间戳（对应之前得到的该帧点云中最早检测点和最晚检测点的时刻）
        //@param[in]: child_frame_id: 接收到的点云数据对应的传感器id
        //@param[in/out]: pose 
        bool Compensator::QueryPoseAffineFromTF2(const uint64_t& timestamp, void* pose,
                                                 const std::string& child_frame_id)
        ```

        通过上述函数就获得了该帧头尾两个时刻到世界坐标系的转换矩阵。

      - `MotionCompensation(msg, msg_compensated, timestamp_min, timestamp_max,            pose_min_time, pose_max_time)`

        ```c++
        //@brief: 主要处理函数，
        //@param[in]: msg（原始接收到的lidar数据）,
        //@param[in]: timestamp_min，timestamp_max 该帧数据头尾两个点的时刻
        //@param[in]: pose_min_time,pose_max_time 对应头尾两个时刻到世界坐标系的转换矩阵
        //@param[in/out]: msg_compensated 输出补偿后的点云数据
        void Compensator::MotionCompensation(
            const std::shared_ptr<const PointCloud>& msg,
            std::shared_ptr<PointCloud> msg_compensated, const uint64_t timestamp_min,
            const uint64_t timestamp_max, const Eigen::Affine3d& pose_min_time,
            const Eigen::Affine3d& pose_max_time) 
        ```

        通过该帧头和尾时刻的世界坐标系的坐标可得到lidar在该帧的时间段内在世界坐标系下相对转动和平移，可以通过线性插值得到中间点近似的旋转和平移。由于lidar的测距精度为~2cm,当距离超过70m，此时角度应小于0.02/70=0.0003rad，认为此时的相对角度是有意义的。

      > 由于lidar各个激光束的依次发射需要时间，因此在这段时间里，可能车辆发生了位移，导致该真点云并不是严格在同一位置检测得到的，因此需要补偿该帧各个点的位置。

  

  - `writer_->Write(point_cloud_compensated);`通过通道`/apollo/sensor/lidar128/compensator/PointCloud2`发布补偿后的点云数据。