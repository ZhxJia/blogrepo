---

title: Apollo中毫米波雷达的初始化配置
categories: 
- apollo
- perception
tags:
- radar
mathjax: true
---

Apollo中毫米波雷达的初始化相关配置

<!--more-->

## 初始化

### 外部配置文件

1. radar component 配置文件
   `modules/perception/onboard/proto/radar_component_config.proto` 

   ```c++
   message RadarComponentConfig {
     optional string radar_name = 1;
     optional string tf_child_frame_id = 2;
     optional double radar_forward_distance = 3;
     optional string radar_preprocessor_method = 4;
     optional string radar_perception_method = 5;
     optional string radar_pipeline_name = 6;
     optional string odometry_channel_name = 7;
     optional string output_channel_name = 8;
   }
   ```

   配置接口：这种读取`.pb.txt`进行配置的方式是通过`GetProtoConfig()`函数读取。
   这种配置文件一般用于组件的初始化配置，具体是通过`cyber`进行`Component`创建时，读取的`config_file`信息:
   可参见文件`cyber/proto/component_conf.proto`,那么对于具体的每个`Component`的conf是怎么加载的呢？这个是通过对应功能组件的`dag`文件进行加载，例如radar的`pb.txt`加载，其存在前向和后向两种组件：

   `modules/perception/production/conf/perception/radar/front_radar_component_conf.pb.txt`

   `modules/perception/production/conf/perception/radar/rear_radar_component_conf.pb.txt`

   以`front_radar`为例，在其`front_radar_component_conf.pb.txt`中的相关配置如下,

   ```c++
   radar_name: "radar_front"
   tf_child_frame_id: "radar_front"
   radar_forward_distance: 200.0
   radar_preprocessor_method: "ContiArsPreprocessor"
   radar_perception_method: "RadarObstaclePerception"
   radar_pipeline_name: "FrontRadarPipeline"
   odometry_channel_name: "/apollo/localization/pose"
   output_channel_name: "/perception/inner/PrefusedObjects"
   ```

   那么该配置文件是通过何种方式加载到的程序中呢？正如前面所提到的，是通过
`production/dag/dag_sreamimg_perception.dag`加载到组件中，作为组件`Initialize`的参数，并最终在`Init()`
   
   中载入。
   
   
   
   **组件配置总结如下**
   
   **Component name**:`RadarDetectionComponent`
   **config**:
   	name: `"FrontRadarDetection"` //组件的node 名称,用于通信
   	config_file:`production/conf/perception/radar/front_radar_component_conf.pb.txt`
   	reader_channel: `"/apollo/sensor/radar/front"`
   	writer_channel: `"/perception/inner/PrefusedObjects"`
   
   **Component name**:`RadarDetectionComponent`
   **config**:
   	name: `"RearRadarDetection"`
   	config_file:`production/conf/perception/radar/rear_radar_component_conf.pb.txt`
   	reader_channel: `"/apollo/sensor/radar/rear"`  
   	writer_channel: `"/Preception/inner/PrefusedObjects"`
   
   上述两个`reader`对应`conti_radar`的`Topic` 。
   
   
   
2. `conti_radar`硬件驱动配置文件

   conti_radar驱动基于Apollo cyber开发，支持continental ARS。

   radar的默认配置:`conf/conti_radar_conf.pb.txt`，在`radar`启动时，会先根据上述配置文件，向can卡发送指令，对`radar`进行配置，当接收到`radar`状态信息与用户信息一致是，才开始解析数据并发送消息。

   `modules/drivers/radar/conti_radar/conf/conti_radar_conf.pb.txt` (也有其对应的proto定义)

   运行过程如下：

   ```bash
   # in docker
   cd /apollo
   source scripts/apollo_base.sh
   # 启动
   ./scripts/conti_radar.sh start
   # 停止
   ./scripts/conti_radar.sh stop
   ```

   对应的信息`Topic`

   **topic name**:` /apollo/sensor/conti_radar`
   **data type**: `apollo::drivers::ContiRadar`
   **channel ID**:` CHANNEL_ID_ONE`
   **proto file**:`modules/drivers/proto/conti_radar.proto` (消息格式)

   

   通过`conti_radar.dag`加载`ContiRadarCanbusComponent`和`ContiRadarCanbusComponent`这两个组件初始化`Initialize()`中所需的配置文件：

   `modules/drivers/radar/conti_radar/conf/radar_front_conf.pb.txt`

   `modules/drivers/radar/conti_radar/conf/radar_rear_conf.pb.txt`

   

### 初始化程序

1. `radar_detection_component`初始化程序

   通过`dag_sreaming_perception.dag`导入组件初始化的参数和配置，分别创建front和rear两个雷达组件，并建立其对应的节点node_，并在此节点上创建一个`Topic`(output_channel_name),同时通过`SensorManager`类查找是否有该传感器信息，以上是基本配置。

   然后进行算法插件的初始化：`InitAlgorithmPlugin()`
   其中主要包括了以下几个部分：

   - hdmap输入初始化
     首先需要判断全局标志位`FLAGS_obs_enable_hdmap_input`是否设置为true,然后进行hdmap_input的初始化

   - preprocessor 预处理模块初始化
     该算法组件的接口文件位于`../radar/lib/interface/base_preprocessor.h`中，用于校正`radar`原始的障碍物信息。

     ```c++
       // @brief: correct radar raw obstacles.
       // @param [in]: raw obstacles from radar driver.
       // @param [in]: options.
       // @param [out]: corrected radar obstacles
       virtual bool Preprocess(const drivers::ContiRadar& raw_obstacles,
                               const PreprocessorOptions& options,
                               drivers::ContiRadar* corrected_obstacles) = 0;
     ```

     根据配置文件中的`radar_preprocessor_method`建立对应的preprocessor实例`ContiArsPreprocessor`。

     

     

     

     > 注意Apollo中感知部分的全局标志位设置位于`../onboard/common_flags_common_flags.cc`
     >
     > 默认启用HdMap:`DEFINE_bool(obs_enable_hdmap_input, true, "enable hdmap input for roi filter");`

   

   

2. `conti_radar_canbus_component`初始化程序

