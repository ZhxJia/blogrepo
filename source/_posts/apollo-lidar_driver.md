---
title: apollo lidar驱动组件
tags:
- 感知
categories:
- lidar
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
firing_data_port: 2368  //velodyne socket 接口
positioning_data_port: 8308 //定位数据 socket结构
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

