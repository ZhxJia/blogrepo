---
title: apollo lidar算法--segmentaion component（一）
tags:
- lidar目标检测
categories:
- apollo
mathjax: true
---



<!--more-->

## 配置文件

SegmentationComponent以128线Velodyne128为例：

```protobuf
 #组件dag配置文件：
 components {
    class_name : "SegmentationComponent"
    config {
      name: "Velodyne128Segmentation"
      config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/velodyne128_segmentation_conf.pb.txt"
      flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
      readers {
          channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
        }
    }
  }
```

参数配置文件：

```protobuf
sensor_name: "velodyne128"
enable_hdmap: true
lidar_query_tf_offset: 0
lidar2novatel_tf2_child_frame_id: "velodyne128"
output_channel_name: "/perception/inner/SegmentationObjects"
```



## 初始化程序

**SegmentationComponent::Init()**

- 1.根据参数配置文件初始化成员变量，创建输出节点 ，通道为`/perception/inner/SegmentationObjects`

  ```
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);
  ```

- 2.`InitAlgorithmPlugin()`

  初始化算法组件（`module/perception/lidar/..`下的各个功能模块）

  - 2.1创建`<LidarObstacleSegmentation>`功能实例`segmentator_` 然后初始化`segmentor_->Init(init_options)`

    - 2.1.1通过`config_manager`获取功能模块的配置信息`model_config`,模块参数配置文件位于：
      `production/conf/perception/lidar/modules/lidar_obstacle_pipeline.config`
      根据该文件中的根路径结合传感器名称可得配置信息如下：

      ```protobuf
      # 定义于lidar/app/proto/lidar_obstacle_segmentation_config.proto
      segmentor: "CNNSegmentation"
      use_map_manager: true
      use_object_filter_bank: true
      ```

    - 2.1.2 创建`<SceneManager>`类实例，并根据`SceneManagerInitOptions`进行初始化。

      ```c++
      // @brief: initialize scene manager with lock
      // @param [in]: initialization options
      // @return: status
      bool SceneManager::Init(const SceneManagerInitOptions& options = SceneManagerInitOptions());
      ```

      - 通过`config_manager`获取该功能模块的配置信息，直接给出配置文件信息：

        ```c++
        # ./data/perception/lidar/models/scene_manager/scene_manager.conf
        service_name: "GroundService"
        service_name: "ROIService"
        ```

        然后由这两个service动态继承`<SceneService>`类创建实例，并进行初始化：`service->Init()`

        - `GroundService::Init()`
          创建`<GroundServiceContent>`类实例`ground_content_ref_`，并通过`config manager`获取配置参数进行初始化，包括以下配置：

          ```protobuf
          # "./data/perception/lidar/models/ground_service"
          roi_rad_x: 120.0
          roi_rad_y: 120.0
          grid_size: 16
          ```

          通过以上配置信息，初始化`ground_content_ref_`实例。

          ```c++
          // note: rows,cols = grid_size
          ground_content_ref_->Init(roi_region_rad_x, roi_region_rad_y, rows, cols);
          ```

          最终得到`resolution,grid_size,...` 地平面上网格的行、列数，网格的尺寸及分辨率，由此初始化`<GroundGrid>`类实例 `grid_.Init(rows_, cols_)`

        - `ROIService::Init()`
          创建`<ROIServiceContent>`类实例`roi_content_ref_`，并通过`config manager`获取配置参数进行初始化，包括以下配置,并设置`roi_content_ref_`的参数为下列值

          ```protobuf
          # ./data/perception/lidar/models/roi_service
          range: 120.0
          cell_size: 0.25
          ```

      - 向Scene Manager中添加初始化后的service：`services_.emplace(name, service);`

    - 2.1.3 初始化点云预处理的功能类`<PointCloudPreprocessor>`实例`cloud_preprocessor_`

      - `PointCloudPreprocessor::Init()`

        通过`config manager`获取该功能的配置参数，直接给出配置文件信息：

        ```protobuf
        # ./data/perception/lidar/models/pointcloud_preprocessor
        filter_naninf_points: false
        filter_nearby_box_points: true
        box_forward_x: 2.0
        box_backward_x: -2.0
        box_forward_y: 3.0
        box_backward_y: -5.0
        filter_high_z_points: false
        z_threshold: 5.0
        ```

    - 2.1.4 初始化地图管理类`<MapManager>`实例`map_manager_`,

      - `MapManager::Init()`

        通过`config manager`获取该功能的配置参数，直接给出配置文件信息：

        ```protobuf
        # ./data/perception/lidar/models/map_manager/map_manager.conf
        update_pose: false
        roi_search_distance: 120.0
        ```

        然后获取`HDMapInput::Instance()`实例`hdmap_input_`，并进行初始化，主要就是加载高清地图的文件路径，此处省略。

    - 2.1.5 根据2.1.1中的segmentor创建分割器接口类`<BaseSegmentation>`派生类`CNNSegmentation`,然后初始化
      `segmentor_->Init(segmentation_init_options)`

      - `CNNSegmentation::Init()`
        初始化该网络模型，首先获取网络的结构，参数，权重的文件路径。

        - `CNNSegmentation::GetConfigs()`
          依然是通过`config manager`和对应的proto获取参数，此处直接给出文件中的参数：

          ```protobuf
          # ./data/perception/lidar/models/cnnseg/cnnseg.conf
          use_paddle: false
          param_file: "./data/perception/lidar/models/cnnseg/velodyne128/cnnseg_param.conf"
          proto_file: "./data/perception/lidar/models/cnnseg/velodyne128/deploy.prototxt"
          weight_file: "./data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel"
          paddle_param_file: "./data/perception/lidar/models/cnnseg/velodyne128/cnnseg_param_paddle.conf"
          paddle_proto_file: "./data/perception/lidar/models/cnnseg/velodyne128/model"
          paddle_weight_file: "./data/perception/lidar/models/cnnseg/velodyne128/params"
          engine_file: "./data/perception/lidar/models/cnnseg/velodyne128/engine.conf"
          ```

          分别对应网络参数，模型的结构和权重三个文件路径。

        - 创建前向推断器，构建网络模型

          ```c++
          inference_.reset(inference::CreateInferenceByName(cnnseg_param_.model_type(),
                                                              proto_file, weight_file,
                                                              output_names, input_names));
          ```

        - 初始化前向推断器

          ```
          input_shape = {1, 8, height_, width_}; 1*8*864*864
          inference_->Init(input_shapes)
          ```

        - 创建输入输出的blob_,用于数据的写入和读取

        - 初始化特征生成器`<FeatureGenerator>`实例，并初始化
          `feature_generator_->Init(feature_param, feature_blob_.get()`

          ```c++
          bool Init(const FeatureParam& feature_param, base::Blob<float>* out_blob);
          ```

          创建变量`std::vector<int> point2grid_`保存每个点在特征图中的一维索引，空间申请为120000
  对应特征图的参数信息为：
          
          ```protobuf
  feature_param {
              width: 864
              height: 864
      point_cloud_range: 90
              min_height: -5.0
              max_height: 5.0
      use_intensity_feature: false
              use_constant_feature: false
          }
          ```
          
        - `CNNSegmentation::InitClusterAndBackgroundSegmentation()`
          初始化聚类和背景分割方法：
        
          - 初始化地平面检测器`ground detector`
            通过`<BaseGroundDetectorRegister>`接口类创建`<SpatioTemporalGroundDetector>`实现类，然后初始化：`ground_detector_->Init(ground_detector_init_options)`
        
            - `SpatioTemporalGroundDetector::Init(）`
              通过`config manager`和对应的proto获取配置参数，此处直接给出配置文件的内容：

              ```protobuf
              #./data/perception/lidar/models/spatio_temporal_ground_detector/
              grid_size: 16
              ground_thres: 0.25
              roi_rad_x: 120.0
              roi_rad_y: 120.0
              roi_rad_z: 120.0
              nr_smooth_iter: 5
      //以上参数为PlaneFitGroundDetectionParam
              use_roi: false
      use_ground_service: true
              ```
        
              通过以上参数初始化`<PlaneFileGroundDetectorParam>`类实例(定义位于common/i_ground)
            
              - `PlaneFitGroundDetectorParam::Validate()` 判断输入参数是否有效
              - 创建`vg_fine_ = new VoxelGridXY<float>()`体素栅格用于对点云数据进行降采样
              - 
            
  - 初始化`roi filter`
            通过`<BaseROIFilter>`接口类创建`<HdmapROIFilter>`实现类然后初始化，用于从点云中提取roi区域的点云：

            - `HdmapROIFilter::Init()`通过`config manager`加载配置参数：
        
      ```protobuf
              #./data/perception/lidar/models/roi_filter/hdmap_roi_filter
              range: 120.0
              cell_size: 0.25
              extend_dist: 0.0
              no_edge_table: false
              set_roi_service: true
              ```
  
              通过以上参数创建`<Bitmap2D>`类实例并初始化。
    
          - 初始化`spp engine`
        获取分割网络的相关输出
        
          - 初始化 `thread worker` ,
          若存在HDMap输入，则进行ROIFilter，最终得到`roi_cloud_`和`roi_world_cloud_`
            roifilter之后则进行地平面检测
        
            ```c++
            ground_detector_->Detect(ground_detector_options, lidar_frame_ref_);
        ```
      
    - 2.1.6  点云分割初始化之后，创建`<ObjectBuilder>`类，用于根据点云创建物体的相关属性。
    
    - 2.1.7 初始化`<ObjectFilterBank>` 用于过滤得到的object,初始化通过`config manager`及对应的proto加载对应的参数：
    
      ```protobuf
      # ./data/perception/lidar/models/object_filter_bank/filter_bank.conf
      filter_name: "ROIBoundaryFilter"
      ```
    
      通过接口类`<BaseObjectFilterRegister>`根据`filter_name`创建类`<ROIBoundaryFilter>`
      对该类进行通过`config mangaer`进行下列参数的加载
    
      ```
      # ./data/perception/lidar/models/object_filter_bank/roi_boundary_filter.conf
      distance_to_boundary_threshold: -1.0
      confidence_threshold: 0.5
      cross_roi_threshold: 0.6
      inside_threshold: 1.0
      ```

**至此，segmentation初始化部分的大体流程就完成了，主要集中于segmentator的初始化，简要总结如下：**

LidarObstacleSegmentation的初始化包括：

- SceneManager类初始化：
  - GroundService初始化
  - ROIService初始化
- PointCloudPreprocessor类初始化 :点云的预处理，包括
- MapManager类初始化 :管理HDMap
- CNNSegmentation网络类初始化
  - inference 推断器（Caffe,RTNet...）初始化 用于网络的前向推断和原始数据的获取
  - FeatureGenerator类初始化 ： 特征提取器初始化
  - SpatioTemporalGroundDetector 类初始化 ：地面检测器和聚类的相关方法
  - HdmapROIFilter 类初始化：根据ROIMap过滤点云
  - SppEngine类初始化：
- ObjectBuilder 类初始化： 用于构建object的相关属性
- ROIBoundaryFilter 类初始化 ：用于过滤object

---









