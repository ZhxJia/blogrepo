---
title: apollo lidar算法--segmentaion component（二）
tags:
- lidar目标检测
categories:
- apollo
mathjax: true
---

apollo中lidar的分割模型，接上篇初始化之后，本篇主要总结apollo segmentation component的`InternalProc`函数内的相关功能实现。从`lidar_error_code.h`中我们可以一窥lidar处理的主要流程阶段：

`Init->Preprocess->MapManager->Segmentaion->ObjectBuilder->ObjectFilter->Classifier->Tracker`

<!--more-->

入口函数：

```c++
// @brief:Lidar相关处理算法
// @param[in]:LidarObstaclesSegmentationOptions 包括了传感器名称和lidar2novatel的外参
// @param[in]:PointCloud 接收的原始点云输入数据
// @param[out]:LidarFrame lidar输出帧信息，包含障碍物识别 跟踪等信息
LidarProcessResult Process(
      const LidarObstacleSegmentationOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame);
```



### 一、预处理：

功能实现类`<PointCloudPreprocessor>`
接口函数：

```c++
  // @brief: preprocess point cloud
  // @param [in]: options 包含lidar到novatel的外参变换矩阵
  // @param [in]: point cloud message
  // @param [in/out]: frame
  // cloud should be filled, required,
  bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame) const;
```

- PointCloudPreprocessor::Preprocess()

  主要就是过滤点云，增加鲁棒性，过滤的选项有以下几个：

  **filter_naninf_points**:过滤点云的坐标为nan 或者超过一定值(kPointInfThreshold=1e3)的点云
  **filter_nearby_box_points**:将离传感器很近的点云过滤掉，注意此处的坐标由各个lidar坐标系转换到novatel坐标系进行同一度量，默认配置的参数范围为：$-2<x<2,-5<y<3$ 。
  **filter_high_z_points:** 将高度大于一定阈值的点云过滤掉(z_threshold_=5.0m)

  上述三个过滤选项可在配置文件中根据需要选择，同时可在配置文件中配置器对应的参数。

  然后将消息帧中过滤后的点云存储到`LidarFrame::cloud`数据结构中，包含点云坐标，时间戳，高度，点的索引，标签等信息。

  - `PointCloudPreprocessor::TransformCloud()`

    ```c++
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud)
    ```

    将局部lidar坐标系下的点云坐标转换到世界坐标系下，并存储到`LidarFrame::world_cloud`数据结构中。

----

### 二、地图管理器MapManger

功能实现类：`<MapManager>`

入口函数：

```c++
  // @brief: update map structure and lidar2world pose
  // @param [in]: options
  // @param [in/out]: frame：the LidarFrame data structure after Preprocess
  // hdmap_struct should be filled, required,
  // lidar2world_pose can be updated, optional,
  bool Update(const MapManagerOptions& options, LidarFrame* frame);
```

- `map_manager_.Update(map_manager_options, frame)`
  通过参数配置文件中的`update_pose`决定是否进行lidar->world姿态的更新。然后通过GetRoiHDMapStruct函数获取当前位置对应的roi区域内的hdmap信息

  - `hdmap_input_->GetRoiHDMapStruct(point, roi_search_distance_,frame->hdmap_struct)`

    ```c++
    // @brief: 获取roi区域内的hdmap结构 包括road polygonss juction_polygons,road_boundary
    // @param [in]: pointd(lidar到世界坐标系的位移向量)，
    // @param [in]: roi_search_distance_(配置参数，roi搜索距离：默认120m)
    // @parma [out]: hdmap_struct_ptr:从hdmap中获取感兴趣区域
    bool HDMapInput::GetRoiHDMapStruct(
        const base::PointD& pointd, const double distance,
        std::shared_ptr<base::HdmapStruct> hdmap_struct_ptr) {
    ```

    首先获取原始的hdmap中指定范围内的道路和道路交叉点的边界:

    - `hdmap_->GetRoadBoundaries(point, distance, &road_boundary_vec,&junctions_vec)`

      ```c++
        /* @brief get all road and junctions boundaries within certain range
         * @param point the target position
         * @param radius the search radius
         * @param road_boundaries the roads' boundaries
         * @param junctions the junctions' boundaries
         * @return 0:success, otherwise failed
         */
        int GetRoadBoundaries(const apollo::common::PointENU& point, double radius,
                              std::vector<RoadROIBoundaryPtr>* road_boundaries,
                              std::vector<JunctionBoundaryPtr>* junctions) const;
      ```

    然后融合两者的结果存入到ROI多边形中，该区域内所有的点均位于世界坐标系下。
    - `MergeBoundaryJunction()`

      ```c++
      // @brief: 融合各个道路和道路交叉点的边界
      // @param[in]: boundary(道路边界)，junctions(交叉点边界)
      // @param[out]: road_boundaries_ptr(道路边界) road_polygons_ptr(道路平面区域)junction_polyjons(路口区域) 
      void HDMapInput::MergeBoundaryJunction(
          const std::vector<apollo::hdmap::RoadRoiPtr>& boundary,
          const std::vector<apollo::hdmap::JunctionInfoConstPtr>& junctions,
          std::vector<base::RoadBoundary>* road_boundaries_ptr,
          std::vector<base::PolygonDType>* road_polygons_ptr,
          std::vector<base::PolygonDType>* junction_polygons_ptr) {...}
      ```

      

    - 根据交叉点过滤道路边界。
      `GetRoadBoundaryFilteredByJunctions()`

      ```c++
      // @brief: 
      // @param[in]: road_boundaries(带过滤的道路边界) junctions_polygons(交叉点区域)
      // @param[out]: flt_road_boundaries_ptr（根据junctions的边界过滤后道路边界）
      bool HDMapInput::GetRoadBoundaryFilteredByJunctions(
          const std::vector<base::RoadBoundary>& road_boundaries,
          const std::vector<base::PointCloud<PointD>>& junctions,
          std::vector<base::RoadBoundary>* flt_road_boundaries_ptr) 
      ```

----

### 三、 分割Segment

功能实现类：继承`<BaseSegmentation>`接口类的`<CNNSegmentation>`类
入口函数：

```c++
  // @brief: segment point cloud and get objects.
  // @param [in]: options
  // @param [in/out]: frame
  // segmented_objects should be filled, required,
  // label field of point cloud can be filled, optional,
  bool Segment(const SegmentationOptions& options,
                       LidarFrame* frame);
```

经过preprocessor和map_manager->update已获得预处理的点云(`cloud`和`world_cloud`)和地图边界信息`HdmapStruct`,此处将填充点云数据帧中的`segmented_objects`。

- **首先**将3d点云映射到2d图像网格中`MapPointToGrid(orginal_cloud_)`

  ```c++
  // @brief: map original point to 2d grid
  // @param [in]: original point
  void CNNSegmentation::MapPointToGrid(
      const std::shared_ptr<AttributePointCloud<PointF>>& pc_ptr) {...}
  ```

  - 遍历原始点云中的每一个点，然后将高度在-5到5m之内的点进行如下处理：
    `GroupPc2Pixel(pt.x, pt.y, inv_res_x, range_, &pos_x, &pos_y);`

    ```c++
    // for axis rotated case
    inline void GroupPc2Pixel(float pc_x, float pc_y, float scale, float range,
                              int* x, int* y) {
      float fx = (range - (0.707107f * (pc_x + pc_y))) * scale; //旋转45度并平移range
      float fy = (range - (0.707107f * (pc_x - pc_y))) * scale;
      *x = fx < 0 ? -1 : static_cast<int>(fx);
      *y = fy < 0 ? -1 : static_cast<int>(fy);
    }
    ```

    根据参数配置文件中的相关信息，俯视图以Lidar为中心范围为(-range=90m,range=90m),对应的网格数为(864,864),这样可得每米对应的格数为inv_res_x=864/180=4.8格/米，由此可以计算每个点云的点对应在哪个格中，上述函数返回了分别对应pc_x,pc_y网格索引，然后将网格索引存储到一维向量中：
    `std::vector<int> point2grid_=pos_y * width_ + pos_x`存储点云中点在网格中的一维索引

- **更近一步**，以网格为基准生成输入到神经网络中计算所需的特征图

  功能实现类`<FeatureGenerator>`
  `feature_generator_->Generate(original_cloud_, point2grid_);` 该部分的函数实现可以通过cpu或者cuda实现：

  ```c++
  // @brief: 生成特征图
  // @param[in]: original_cloud_ 原始lidar点云,point2grid_(点云中的点对应网格的一维索引) 
  void Generate(const base::PointFCloudPtr& pc_ptr,
                  const std::vector<int>& point2grid) {
  #ifndef PERCEPTION_CPU_ONLY
      GenerateGPU(pc_ptr, point2grid);
  #else
      GenerateCPU(pc_ptr, point2grid);
  #endif
    }
  ```

  该部分用于提取输入到网络模型中的特征图，特征图信息通过以下几个步骤获取：

  - 初始化特征属性：
    最大高度(max_height)，平均高度(mean_height)，最高点对应的强度(top_intensity)，平均强度(mean_intensity),以及网格中点云的密度(count)，以及对应该网格是否为空的nonempty_data,

  - 计算特征属性：

    遍历点云，根据point2grid_可获得点云对应的网格一维索引，获取网格对应所有点云的最大高度以及平均高度，若配置参数中启用intensity作为特征（`use_intensity_feature=Ture`）则同时获取最大高度对应的强度和平均强度特征，同时记录每个网格中对应的点的个数,若是网格中没有点，则用最大高度记为0

  - 每个网格的点的个数取对数(归一化)作为网格密度的特征，

    ```c++
      float LogCount(int count) {
        if (count < static_cast<int>(log_table_.size())) {
          return log_table_[count]; //查询log查询表
        }
        return logf(static_cast<float>(1 + count));
      }
    ```

    将提取的特征属性分别存放以下指针中，注意指针指向`feature_blob`对应的数据地址
    `max_height_data_,mean_height_data_,top_intensity_data_,mean_intensity_data,count_data_,nonempty_data`

    **此处计算的特征图通道数最多为6，再加上初始化时已经填充的`distance_data`和`direction_data`则最多有8个通道的输入特征图，注意max_height,mean_height,count_data，nonempty_data必须之外，在程序中其余特征是通过配置文件参数决定是否使用,程序默认只使用四个必须的通道**

  >  参考论文 [MV3D][https://www.baidu.com/link?url=Ly_VMVTyYOUZVTepDik7Lt6L8YVF1eZjGAYdxX4VsXZH0RRs3jVs3QrLaxGk8W6H&wd=&eqid=ac4db69d0000b034000000025e97375a]中Bird’s Eye View Representation.章节的相关描述，MV3D中以$min(1.0,\frac{log(N+1)}{log(64)})$作为网格密度特征的衡量方式。

- **然后**，在获得了对应的输入特征图`(1*4*864*864)`之后，就可以进行模型的前向推断啦
  `inference_->Infer()`
  网络模型的输出包括：
  类别得分`class_pt_blob:"class_score"`，置信度得分`confidence_pt_blob:"confidence_score"`
  实例`instace_pt_blob:"instace_pt"`,分类`category_pt_blob:"category_score"`,朝向`heading_py_blob:"heading_pt"`
  障碍物高度`height_pt_blob:"height_pt"`
  **通过神经网络输出，可以得到每个单元格的CNN预测属性**
- **最后** 进行后处理聚类：





> CnnSeg网络的相关解析可以参考：https://zhuanlan.zhihu.com/p/35034215







