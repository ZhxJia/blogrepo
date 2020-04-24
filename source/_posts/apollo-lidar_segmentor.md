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
LidarProcessResult LidarObstacleSegmentation::Process(
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

经过preprocessor和map_manager->update已获得预处理的点云(`cloud`和`world_cloud`)和地图边界信息`HdmapStruct`,此处将填充点云数据帧中的`segmented_objects`，方法是通过聚类点云形成障碍物。

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
    计算的8个统计量：

    1. 单元格中点的最大高度
    2. 单元格中最高点的强度
    3. 单元格中点的平均高度
    4. 单元格中点的平均强度
    5. 单元格中的点数
    6. 单元格中心相对于原点的角度
    7. 单元格中心与原点之间的距离
    8. 二进制值标示单元格是空还是被占用

  >  参考论文 [MV3D][https://www.baidu.com/link?url=Ly_VMVTyYOUZVTepDik7Lt6L8YVF1eZjGAYdxX4VsXZH0RRs3jVs3QrLaxGk8W6H&wd=&eqid=ac4db69d0000b034000000025e97375a]中Bird’s Eye View Representation.章节的相关描述，MV3D中以$min(1.0,\frac{log(N+1)}{log(64)})$作为网格密度特征的衡量方式。

- **然后**，在获得了对应的输入特征图`(1*4*864*864)`之后，就可以进行模型的前向推断啦
  `inference_->Infer()`
  网络模型的输出包括：
  类别得分`class_pt_blob:"class_score"`，置信度得分`confidence_pt_blob:"confidence_score"`
  偏移`instace_pt_blob:"instace_pt"`,分类`category_pt_blob:"category_score"`,朝向`heading_py_blob:"heading_pt"`
  障碍物高度`height_pt_blob:"height_pt"`
  **通过神经网络输出，可以得到每个单元格的CNN预测属性**
  基于FCNN的预测，Apollo获取了每个单元格的四个预测信息，分别用于之后的障碍物聚类和后处理：

  | 单元格属性           | 单元格属性    | 用途       |
  | -------------------- | ------------- | ---------- |
  | 中心偏移             | center offset | 障碍物聚类 |
  | 对象性(是否为object) | objectness    | 障碍物聚类 |
  | 可信度(positiveness) | positiveness  | 后处理     |
  | 对象高度             | object height | 后处理     |

  

- **最后** 进行后处理聚类：
  `GetObjectsFromSppEngine(&frame->segmented_objects);`

  ```c++
  // @brief: 后处理实现点云聚类
  // @param[out]: objects (点云数据帧中的segmented_objects数据)
  void CNNSegmentation::GetObjectsFromSppEngine(
      std::vector<std::shared_ptr<Object>>* objects){...}
  ```

  功能实现类：`<SppEngine>`

  1. 向`SppEngine`的数据结构`<SppData>`中传入点云对应的一维网格索引`grid_indices`，即特征图。

  2. 进行前景分割,对点云进行聚类得到各个物体的cluster

     ```c++
       // @brief: process foreground segmentation
       // @param [in]: point cloud (原始点云数据)
       // @return: size of foreground clusters
       size_t ProcessForegroundSegmentation(
           const base::PointFCloudConstPtr point_cloud);
     ```

     此处 处理的数据结构类为：`<CloudMask>`

     - 调用私有函数在输入网格特征图上进行聚类：

       ```c++
         // @brief: process clustering on input feature map
         // @param [in]: point cloud
         // @param [in]: point cloud mask
         size_t ProcessConnectedComponentCluster(
             const base::PointFCloudConstPtr point_cloud, const CloudMask& mask);
       ```

       - 在该函数中，通过`<SppCCDetector>`这个功能类检测cluster。

         ```c++
         // @brief: detect clusters
         // @param [out]: label image
         // @return: label number 检测到的物体类别标签的数量
         size_t SppCCDetector::Detect(SppLabelImage* labels) {...}
         ```

         其中参数 labels 为数据结构类`<SppLabelImage>`

         - `BuildNodes(0,rows_)` 通过给定特征图行数创建`node`矩阵 ，根据网络输出的
           `category_pt_blob`对应每一个节点是否是object的概率，`instance_pt_blob`对应着每个节点在行和列方向上的偏移（网络的输出为米，需根据scale转换为偏移的网格数），创建节点并设置节点的`is_object`状态。**这里的偏离center offset实际有两层，分别是行方向上的偏移和列方向上的偏移，最终将二者合并成整个特征图的一维索引保存到对应节点的center_node属性中**

         > 这里的`<Node>`数据结构类型包含的是一个16位的数据状态status,表示一个网格节点的状态，将
         > `node_bank,traversed,is_center,is_object`等属性压缩到这一个uint16数据中,排列如下
         > `|is_center(1bit)|is_object(1bit)|traversed(3bit)|node_rank(11bit)|`

         - `TraverseNodes()` 遍历创建的节点矩阵，如果节点对应的is_object = 1,构建节点之间的对应关系：
           遍历is_object=1的节点，根据该节点的center_node偏移不断往下遍历直到遇到已经遍历过的点则停止，将这些点的is_center属性赋值为true。然后将各个遍历过程中经过的节点的traversed属性置为1，并将各个节点的parent属性统一设置为最终(遍历停止)节点的parent(即为网格一维索引)

         - `UnionNodes()`将相邻的节点进行组合。采用压缩的联合查找算法(Union Find algorithm)有效查找连接组件，每个组件都是候选障碍物集群。

           > 参考算法”并查集“

         - `ToLabelMap(labels)` 将障碍物cluster收集到`<SppLabelImage>`中的数据存储结构`<SppCluster>`中，设置label image中各个cluster的标签id，从1开始，将相同根节点的网格(**程序中称为pixel**)添加到同一个cluster中。

       - 然后对获取的到的cluster进行过滤(根据网络输出的confidence_pt_blob,category_pt_blob,以及
         confidence的阈值默认0.1，objectness的阈值0.5进行判断)

         ```c++
         // @brief: filter clusters, given confidence map
         // @param [in]: confidence_map of the same size
         // @param [in]: confidence threshold
         void SppLabelImage::FilterClusters(const float* confidence_map,
                                            const float* category_map,
                                            float confidence_threshold,
                                            float category_threshold) 
         ```

         通过将cluster中所有的pixel对应的confidence或者category的分数和取平均作为cluster的置信度，根据阈值判断是否有效，并对clusters_ 进行过滤，并更新labels_ 中的标签，将无效的cluster的标签置为0(背景)

       - 根据网络输出的class map计算每个cluster的类别

         ```c++
           // @brief: calculate class for each cluster, given class map
           // @param [in]: class_map of the same size
           // @param [in]: class number default:5
           void SppLabelImage::CalculateClusterClass(const float* class_map, size_t class_num);
         ```

         默认类别数量是有5类：`UNKNOWN,SAMLLMOT,BIGMOT,NONMOT,PEDESTRIAN` 
         网络的输出`class_map`共有class_num层,每一层大小为width*height(网格大小)

       - 根据网络输出的heading_data计算每个cluster的朝向

         ```c++
           // @brief: calculate heading (yaw) for each cluster, given heading map
           // @param [in]: heading_map of the same size
           void SppLabelImage::CalculateClusterHeading(const float* heading_map);
         ```

         网络输出的heading_map 是由x朝向和y朝向两层组成，根据x方向和y方向的位置朝向通过arctan(y/x)计算得到yaw轴角度。

       - 根据网络输出的top_z_map计算cluster的高度

         ```c++
           // @brief: calculate top_z for each cluster, given top_z map
           // @param [in]: top_z_map of the same size
           void SppLabelImage::CalculateClusterTopZ(const float* top_z_map);
         ```

       - 根据label_image计算的clusters_ 对齐spp_cluster_list中的clusters_，然后向cluster中添加点云中的点的点的信息，即2d->3d

         ```c++
           // @brief: add an 3d point sample
           // @param [in]: cluster id
           // @param [in]: 3d point
           // @param [in]: point height above ground
           // @param [in]: point id
           void SppClusterList::AddPointSample(size_t cluster_id, const base::PointF& point,
                               	float height, uint32_t point_id);
         ```

       - 清除空的cluster

         ```c++
           // @brief: remove empty cluster from clusters
           void SppClusterList::RemoveEmptyClusters();
         ```

  3. 进行**背景分割**，首先需要同步线程，然后将roi点云中的高度拷贝到原始点云中，并将原始点云中的标签修改为对应roi_id的`LidarPointLabel::GROUND`，然后移除ground对应的点points。

     ```c++
       // @brief: remove ground points in foreground cluster
       // @param [in]: point cloud
       // @param [in]: roi indices of point cloud
       // @param [in]: non ground indices in roi of point cloud
       size_t SppEngine::RemoveGroundPointsInForegroundCluster(
           const base::PointFCloudConstPtr full_point_cloud,
           const base::PointIndices& roi_indices,
           const base::PointIndices& roi_non_ground_indices);
     ```

  4. 将clusters的相关属性添加到`<object>`数据结构中，例如cluster各个类别的概率和对应的object类型：
     object中的`UNKNOWN,PEDESTRIAN,BICYLE,VEHICLE`
     分别对应cluster中的`META_UNKNOW,META_PEDESTRIAN,META_NOMOT,META_SMALLMOT+META_BIGMOT`
     然后将cluster的朝向信息复制到`<object>`的`theta`,`direction`属性。

----

### 四、 障碍物边框构建

对象构建器组件为检测到的障碍物建立一个边界框。
接口函数：

```c++
  // @brief: calculate and fill object size, center, directions.
  // @param [in]: ObjectBuilderOptions.
  // @param [in/out]: LidarFrame*.
  bool ObjectBuilder::Build(const ObjectBuilderOptions& options, LidarFrame* frame);
```

- 首先对每一个检测得到的物体计算2D多边形凸包：

  ```c++
    // @brief: calculate 2d polygon.
    //         and fill the convex hull vertices in object->polygon.
    // @param [in/out]: ObjectPtr.
    void ComputePolygon2D(
        std::shared_ptr<apollo::perception::base::Object> object);
  ```

  - 计算物体包含点云的最小最大值

    ```c++
      // @brief: calculate 3D min max point
      // @param [in]: point cloud.
      // @param [in/out]: min and max points.
      void ObjectBuilder::GetMinMax3D(const apollo::perception::base::PointCloud<
                           apollo::perception::base::PointF>& cloud,
                       Eigen::Vector3f* min_pt, Eigen::Vector3f* max_pt);
    ```

    根据该物体包含的点云，得到该点云中x,y,z方向向最小值`min_pt`和最大值`max_pt`。

  - 根据x,y,z方向上的最大值和最小值，计算和填充默认的多边形的属性值

    ```c++
     // @brief: calculate and fill default polygon value.
      // @param [in]: min and max point.
      // @param [in/out]: ObjectPtr.
      void ObjectBuilder::SetDefaultValue(
          const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt,
          std::shared_ptr<apollo::perception::base::Object> object);
    ```

    根据min_pt和max_pt计算默认的`center,size(LWH),direction,polygon(4)`

  - 判断输入的点云是否位于一条直线上，如果是的话加入轻微扰动

    ```c++
      // @brief: decide whether input cloud is on the same line.
      //         if ture, add perturbation.
      // @param [in/out]: pointcloud.
      // @param [out]: is line: true, not line: false.
      bool ObjectBuilder::LinePerturbation(
          apollo::perception::base::PointCloud<apollo::perception::base::PointF>*
              cloud);
    ```

    判断三点是否共线，$x_1y_2=x_2y_1$ ，若共线则在坐标上添加一个小扰动0.001

  - 根据输入点云得到多边形

    ```c++
      // @brief: main interface to get polygon from input point cloud
      bool GetConvexHull(const CLOUD_IN_TYPE& in_cloud,
                         CLOUD_OUT_TYPE* out_polygon) {
        SetPoints(in_cloud);//将点云复制到类内成员变量
        if (!GetConvexHullMonotoneChain(out_polygon)) {
          return MockConvexHull(out_polygon);
        }
        return true;
      }
    ```

    - 利用二维凸包算法 计算凸包

      ```c++
        // compute convex hull using Andrew's monotone chain algorithm
        bool common::GetConvexHullMonotoneChain(CLOUD_OUT_TYPE* out_polygon);
      ```

      >1. 排序，根据某个坐标轴为主进行排序
      >2. 从x最小的点开始
      >3. 利用类似Graham’s Scan算法，利用栈去寻找下半包
      >4. 从x最大的点开始
      >5. 利用类似Graham’s Scan算法，利用栈去寻找上半包
      >6. 两个半包结合，即是整个凸包
      >     （上半包和下半包可以合并成一个while函数）

      针对一些退化的情况，例如点云中的点小于3等情况，通过以下方式获取凸包：

      ```c++
        // mock a polygon for some degenerate cases
        bool MockConvexHull(CLOUD_OUT_TYPE* out_polygon);
      ```

- 计算凸包中心和大小：

  ```c++
    // @brief: calculate the size, center of polygon.
    // @param [in/out]: ObjectPtr.
    void ComputePolygonSizeCenter(
        std::shared_ptr<apollo::perception::base::Object> object);
  ```

  - 计算边界框的大小和中心点,输入包括点云，物体方向，输出边界框的大小和中心点

    ```c++
    // @brief calculate the size and center of the bounding-box of a point cloud
    // old name: compute_bbox_size_center_xy
    template <typename PointCloudT>
    void CalculateBBoxSizeCenter2DXY(const PointCloudT &cloud,
                                     const Eigen::Vector3f &dir,
                                     Eigen::Vector3f *size, Eigen::Vector3d *center,
                                     float minimum_edge_length = FLT_EPSILON) {
    ```

  - 计算并填充其余的一些物体多边形信息，如时间戳和anchor_point

    ```c++
      // @brief: calculate and fill timestamp and anchor_point.
      // @param [in/out]: ObjectPtr.
      void ComputeOtherObjectInformation(
          std::shared_ptr<apollo::perception::base::Object> object);
    ```

    将object的中心作为anchor_point,object所包含点云所有点的平均时间作为物体的最新测量时间。
    更新`object->anchor_point`,`object->latest_tracked_time`

----

### 五、过滤检测得到的object

接口函数：

```c++
  // @brief: filter objects
  // @param [in]: options
  // @param [in/out]: frame
  // segmented_objects should be valid, and will be filtered,
  bool ObjectFilterBank::Filter(const ObjectFilterOptions& options, LidarFrame* frame);
```

通过`FilterBank`中存在的filter过滤器对检测得到的object进行过滤，注意此处filterbank中可能存在不止一种过滤器,此处在初始化配置是添加的过滤器名称为:`ROIBoundaryFilter` 通过高精度地图中感兴趣的区域进行过滤。

如果高精度地图中没有对应的roi polygons，则跳过该步骤，并令object中的：
`object->lidar_supplement.is_in_roi=true`
否则则进行过滤：

1. 首先，`FillObjectRoiFlag(options, frame);`确定object是否位于roi的交界处。

   ```c++
     // @brief: fill is_in_roi in lidar object supplement
     void ROIBoundaryFilter::FillObjectRoiFlag(const ObjectFilterOptions& options, LidarFrame* frame);
   ```

2. 然后，在世界坐标系中构建polygon(将原lidar坐标系中的polygon中的点转换到世界坐标系中的点)

   ```c++
     // @brief: given input objects, build polygon in world frame
     void ROIBoundaryFilter::BuildWorldPolygons(const ObjectFilterOptions& options,
                             const LidarFrame& frame);
   ```

3. 然后，过滤在道路边界之外的object：

   ```c++
     // @brief: filter outside objects based on distance to boundary
     void FilterObjectsOutsideBoundary(const ObjectFilterOptions& options,
                                       LidarFrame* frame,
                                       std::vector<bool>* objects_valid_flag);
   ```

4. 然后，过滤在道路边界内的object,置信度<=0.11。

   ```c++
     // @brief: filter inside objects based on distance to boundary
     void FilterObjectsInsideBoundary(const ObjectFilterOptions& options,
                                      LidarFrame* frame,
                                      std::vector<bool>* objects_valid_flag);
   ```

5. 最后，通过confidence对在roi边界处 或者roi外部的object的confidence<0.5进行过滤：

   ```c++
     // @brief: filter objects based on position and confidence
     void FilterObjectsByConfidence(const ObjectFilterOptions& options,
                                    LidarFrame* frame,
                                    std::vector<bool>* objects_valid_flag);
   ```

----

**最后将检测结果通过通道`"/perception/inner/SegmentationObjects"`输出该部分得到的障碍物检测信息，对应的消息类型是：`<LidarFrameMessage>`（位于lidar_inner_component_message.h中 即内部消息类型定义）** 

> CnnSeg网络的相关解析可以参考：https://zhuanlan.zhihu.com/p/35034215
>
> https://www.jianshu.com/p/95a51214959b
> https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md

扫描线算法：https://www.jianshu.com/p/d9be99077c2b





