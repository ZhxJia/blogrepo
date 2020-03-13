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

   - **hdmap**输入初始化
     首先需要判断全局标志位`FLAGS_obs_enable_hdmap_input`是否设置为true,然后进行hdmap_input的初始化

   - **preprocessor** 预处理模块初始化
     该算法组件的接口文件位于`../radar/lib/interface/base_preprocessor.h`中，用于校正`radar`原始的障碍物输出信息。

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

     这里`ContiArsPreprocessor`的配置文件加载用的是`config_manager.cc`中的`ConfigManager`这个类（这个类通过查找`perception_gflags.cc`中声明的路径"production/conf/"来载入配置文件`*.conf`）来载入配置文件
   `production/conf/perception/radar/modules/conti_ars_preprocessor.config`中的参数
     "delay_time = 0.07"

     > 注意Apollo中感知部分的全局标志位设置位于`../onboard/common_flags_common_flags.cc`
   >
     > 默认启用HdMap:`DEFINE_bool(obs_enable_hdmap_input, true, "enable hdmap input for roi filter");`
     

   

   - **perception**感知模块初始化
     该算法组件的接口文件位于`../radar/lib/interface/base_radar_obstacle_perception.h`中，用于校正`radar`感知主体算法。

     ```c++
       virtual bool Perceive(const drivers::ContiRadar& corrected_obstacles,
                             const RadarPerceptionOptions& options,
                             std::vector<base::ObjectPtr>* objects) = 0;
     ```

     同样由组件配置文件的参数`radar_perception_method: "RadarObstaclePerception"`确定实例化的类。

     类似地，该功能模块的参数配置文件为`*.config`类型，因此也通过`ConfigManager`类来进行参数加载,

     对于不同的radar，需要加载不同的参数，通过组件配置类中的`radar_pipeline_name`来确定具体加载的参数配置文件：
     	对于rear_radar:
     	`radar_pipeline_name: "RearRadarPipeline"`对应于配置文件`rear_radar_pipeline.config`
     	对于`front_radar`:

     ​	`radar_pipeline_name:"FrontRadarPipeline"`对应配置文件`front_radar_pipeline.config`

     根据对应的配置文件加载参数到`RadarObsaclePerception`类中,以`front_radar`为例:

     ```c++
     model_configs {
       name: "RearRadarPipeline"
       version: "1.0.0"
       string_params {
         name: "Detector"
         value: "ContiArsDetector"
       }
       string_params {
         name: "RoiFilter"
         value: "HdmapRadarRoiFilter"
       }
       string_params {
         name: "Tracker"
         value: "ContiArsTracker"
       }
     }
     ```

     其中包含了三个参数分别对应`Percepiton`中的三个主要算法的实现类:

     - **Detector** -> **ContiArsDetector**
     - **RoiFilter** -> **HdmapRadarRoiFilter**
     - **Tracker** -> **ContiArsTracker**  

     对应的算法接口如下(`modules/perception/radar/lib/interface/..`)：

     ```c++
       // @brief: detect the objects from the corrected obstacles
       // @param [in]: corrected obstacles.
       // @param [in]: options.
       // @param [out]: detected objects.
       virtual bool Detect(const drivers::ContiRadar& corrected_obstacles,
                           const DetectorOptions& options,
                           base::FramePtr detected_frame) = 0;
     ```

     ```c++
       // @brief: fliter the objects outside the ROI
       // @param [in]: options.
       // @param [in / out]: origin total objects / the objects in the ROI.
       virtual bool RoiFilter(const RoiFilterOptions& options,
                              base::FramePtr radar_frame) = 0;
     ```

     ```c++
       // @brief: tracking objects.
       // @param [in]: current object frame.
       // @param [in]: options.
       // @param [out]: current tracked objects frame.
       virtual bool Track(const base::Frame &detected_frame,
                          const TrackerOptions &options,
                          base::FramePtr tracked_frame) = 0;
     ```

     `Detect`和`RoiFilter`初始化过程较为简单，而`Track`的初始化则稍显复杂，其又包括了两个比较重要的实现类

     `matcher`和`track_manager`。

     ​	进一步地，`ContiArsTracker`的初始化的参数配置也是由`*.config`加载的，由`ConfigManager`加载文件

     `production/conf/../radar/modules/conti_ars_tracker.config`,参数包括：

     ```c++
     model_configs {
       name:"ContiArsTracker"
       version: "1.0.0"
       double_params {
         name: "tracking_time_window"
         value: 0.06
       }
       string_params {
         name: "macher_name"
         value: "HMMatcher"
       }
       string_params {
         name: "chosen_filter"
         value: "AdaptiveKalmanFilter"
       }
       integer_params {
         name: "tracked_times_threshold"
         value: 3
       }
       bool_params {
         name: "use_filter"
         value: false
       }
     }
     ```

     **初始化`HMMatcher`:**
     		同样由`ConfigManager`根据模型名称加载配置文件`hm_matcher.config`:

     ```protobuf
     model_configs {
       name: "HMMatcher"
       version: "1.0.0"
       string_params {
         name: "root_path"
         value: "./data/perception/radar/models/tracker"
       }
     }
     ```

     该路径下对应的文件`hm_matcher.conf`对应参数载入到`HMMatcher`类内成员变量。

     ```c++
     max_match_distance : 2.5
     bound_match_distance : 10.0
     ```

   最后，进行`TransformWrapper`类的初始化,包括`radar2world`和`radar2novatel`,和坐标转换相关。

   需要注意的是，`radar`这里订阅了`localization`的消息，消息通道名称为：`/apollo/localization/pose`,

   `MsgBuffer<LocalizationEstimate> localization_subscriber_;`由MsgBuffer根据Component中的
   `odometry_channel_name`创建`node`,名称为`pose_radar_front_subscriber`并由此node创建reader，以及消息的回调函数,在消息回调函数`MsgCallback`中，将消息及对应的时间戳存入到`buffer_queue_`中。

   **至此，`radar_detection_component`**的初始化完成,主体算法将在`Proc`中周期执行。

   ​				

2. `conti_radar_canbus_component`初始化程序

