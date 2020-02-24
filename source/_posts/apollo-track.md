---

title: Apollo 中的检测算法
categories:
- Apollo
tags:
- track
mathjax: ture
---

Apollo中的检测算法

<!--more-->

跟踪的基本成员单位为封装object类为TrackObject类，最后`Target`类结构中包含`TrackObject`类及相关的估计方法。

## tracker_->Predict(frame):预测部分

在新图像中预测候选障碍物 将输出赋值给frame->proposed_objects

## extractor_->Extract(frame):特征提取部分

对每个检测到的对象进行特征提取

```c++
struct FeatureExtractorLayer {
  std::shared_ptr<inference::Layer<float>> pooling_layer;
  std::shared_ptr<base::Blob<float>> rois_blob;
  std::shared_ptr<base::Blob<float>> top_blob;
};
```

```c++
   feature_extractor_layer_ptr->pooling_layer->ForwardGPU(
        {feat_blob_, feature_extractor_layer_ptr->rois_blob},
        {frame->track_feature_blob});
```

其中`feat_blob_`的现行定义是:

```protobuf
feat_stride: 32
extractor {
  feat_blob: "conv4_3"
  feat_type: ROIPooling
  roi_pooling_param {
      pooled_h: 3
      pooled_w: 3
      use_floor: true
  }
}
```

这样根据feat_blob_以及rois_blob(由bbox位置确定)进行`ROIPooling`将最终pool得到的最大值存到track_feature_blob中。

## tracker_->Associate2D(frame):根据2D信息关联

### 该部分的主要成员变量

1. 成员类帧列表:`FrameList` 包装了`CameraFrame`类

   ```c++
     int frame_count_ = 0; //添加帧的数量
     int capability_ = 0;	//容量(14)表示14帧后开始覆盖
     std::vector<CameraFrame *> frames_;
   ```

2. 相似性map:`SimilarMap` 存储blob的嵌套vector

   ```c++
     std::vector<std::vector<std::shared_ptr<base::Blob<float>>>> map_sim;
     int dim;//=omt_param_.img_capability()=14
   ```

3. 相似性计算`BaseSimilar`  `std::shared_ptr<BaseSimilar> similar_`

![](apollo-track/2.png)

​	其中GPUSimilar用到了BLAS(线性代数)库:https://blog.csdn.net/cocoonyang/article/details/58602654?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task

4. `Target`类包含了跟踪物体的id和相关的状态信息,`tracked_objects`包含了被跟踪的物体vector,是`TrackObject`类
   `Target`类为期望得到检测框匹配的跟踪物体

   ![](apollo-track/4.png)

5. `TrackObject`类则封装了跟踪目标`Object`类，记录该跟踪目标对应的帧id和传感器名称,是跟踪目标的最小封装单位

   ```c++
   struct TrackObject {
     PatchIndicator indicator; //保存了传感器的名称和目标在帧中的id
     double timestamp;
     base::BBox2DF projected_box;//由检测得到的原2dbox　经projected_matrix投影得到障碍物的bbox 这个矩阵是啥意思不太清楚 narrow to obstacle projected_matrix(应该是为了将不同的相机的bbox统一到一个尺度)
     base::ObjectPtr object;
   };
   ```

6. `PatchIndicator`类主要包含了两个变量,重载运算符"=="为frame_id和patch_id分别相等

   ```c++
     int frame_id;//为该检测目标对应的是第几帧(frame id )
     int patch_id;//该检测目标在该帧中的id(object id)
     std::string sensor_name;//记录检测到的传感器名称
   ```

7. `ObstacleReference`类包含了与模板相关的参数：

   ![](apollo-track/5.png)

8. `reference`类

   ```c++
   struct Reference {
     float area = 0.0f;
     float k = 0.0f;
     float ymax = 0.0f;
   };
   ```

9. `CameraGroundPlaneDetector`类，地平面检测的相关方法

   ![](apollo-track/6.png)

### 2D关联算法的实现

1. 计算各帧与当前帧的余弦相似性:（第5步target和det_obj的Apperance的匹配分数会用到）
2. 从已跟踪物体列表中去除最早帧之前的帧中检测到的目标
3. 根据当前帧的检测物体，得到当前的检测跟踪列表

  ```c++
  for(...){
      ...
  	ProjectBox(frame->detected_objects[i]->camera_supplement.box,
               frame->project_matrix, &(track_ptr->projected_box));//图像2dbox投影到障碍物坐标系
  	track_objects.push_back(track_ptr);
  }
  ```

4. 校正当前帧检测目标的三维尺寸，执行了四种校正方式：分别为模板，参考地平面，标定，历史信息

  ```c++
  reference_.CorrectSize(frame); //
  ```

  - 根据模板的最小最大尺寸校正检测目标的最小最大尺寸。(TypeRefinedBy**Template**)

  - 根据障碍物参考校正检测目标的尺寸,可作为参考的目标类型类`CAR,VAN`,同时还需要一系列附加条件，可详见`ObstacleReference::UpdateReference(...)`

    (1). 根据sensor_name的不同 初始化ground_estiomator,执行`GameraGroundPlaneDetector`类的初始化:

    ```c++
    SyncGroundEstimator(sensor, frame->camera_k_matrix,
                              static_cast<int>(img_width_),
                              static_cast<int>(img_height_));
    ```

    该函数实际上执行`GameraGroundPlaneDetector`类的初始化,有相机内参相关参数初始化

    ```c++
    ground_estimator.Init(k_mat, img_width, img_height, common::IRec(fx));
    ```

    (2). 

    ```c++
    ground_estimator.GetGroundModel(l)
    ```

    通过相机pitch和相机height(通过calibration获取)获取平面$$a*X+b*Y+c*Z+d=0$$ （相机坐标系）

  - 通过标定服务校正高度h

  - 通过历史参考校正高度h

  

  

  ![template](apollo-track/21.png)

>
> 此处矫正的三维尺寸在后面的`Transform`部分是否也被采用

1. 生成假设

   ```c++
   // @brief: 评估新检测目标与targets的相似性
   // @param [in]: track_objects :该帧检测器新检测到的目标
   // @param [in/out]: 
   // output:target匹配第(frame_id)帧中的第几个检测物(track_object),
   //        并将该track_object加入到target的tracked_objects中
   GenerateHypothesis(track_objects);
   ```

   其中对应的主要成员变量类型`Hypothesis`的定义为:（此为target和object的）

   ![](apollo-track/666.png)

   处理函数：

   ![](apollo-track/13.png)

   得到target与对应det_obj的匹配分数后，根据分数由大到小分配,最终将检测物体(det_obj)加入到匹配分数最高的target的tracked_objects中。

   ```c++
   target.Add(det_obj); 
   //将det_obj添加到给定的target的tracked_objects中,并将该target的lost_age清零
   ```

   除了上面的四个分数，最终score还需要加上不同物体类型切换的代价值。

   ![type_change_cost](apollo-track/10.png)

2. 创建新跟踪目标(target)

   ```c++
   int new_count = CreateNewTarget(track_objects);
   ```

   由`track_objects`创建新的跟踪目标的条件有：

   	1. 首先该det_obj与现有的target的匹配分数很小，即该det_obj与现有target匹配失败
    	2. 该det_obj的box需要是有效的(bbox宽高大于20同时小于图像宽高)
    	3. 该det_obj的box矩形没有被某个target的bbox覆盖 ,此处target的bbox指tracked_objects中最近检测的bbox
    	4. 在上述条件都满足的前提下，需满足det_obj的高度大于最小模板的高度,也可以是未知类型(此时高度的模板未知)

7. 超过存活周期(lost_age>5)的target,否则执行(即正常跟踪的target)下列方法，通过`latest_object`进行相应更新.并调用对应的`滤波方法`进行预测

   ```c++
   target.UpdateType(frame);
   target.Update2D(frame);
   ```

   - **UpdateType():**

     对于未丢失检测(lost_age=0)的target:

     1. 计算该target的最近检测目标(latest_object->object)的高度与模板中等尺寸的高度接近程度,接近程度由高斯分布衡量:

     ```c++
     //~N(mu=1.0,sigma=0.3)  反映了object的box_height与模板中等高度的接近程度
     float alpha = gaussian(
             rect.height /
                 (50 * (kMidTemplateHWL.at(object->sub_type).at(0) + 0.01f)),
             1.0f, target_param_.type_filter_var());
     //该target同类型的probs会往上叠加(在不同帧之间)，因为每帧该target都有一个对应的latest_object
     type_probs[static_cast<int>(object->sub_type)] += alpha; 
     ```

     根据上述最大的`type_probs`更新target的对应类型

     2. 将上述(alpha, object->size(0), object->size(1),object->size(2))组成4维向量，将此测量量添加到该target的`MaxNMeanFilter world_lwh`及`MeanFilter world_lwh_for_unmovable`中：

     ````c++
     world_lwh.AddMeasure(size_measurement); //MaxNMeanFilter window=15 根据alpha由大到小
     world_lwh_for_unmovable.AddMeasure(size_measurement); //测量值的均值和方差
     ````

     ![MaxNMeanFilter](apollo-track/19.png)

     这一步将得到该target 的15次测量值，按照alpha由大到小排序，并取平均更新3d object size,其中的object为该target的lastest_obj

     ```c++
         if (object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
           object->size =
               world_lwh_for_unmovable.get_state().block<3, 1>(1, 0).cast<float>();
         } else {
           object->size = world_lwh.get_state().block<3, 1>(1, 0).cast<float>();
         }
     ```

     > camera:
     >
     > 6mm : reliable z is 40(m) ,intrinsic(f) is approximate is 2000(像素)
     >
     > 12mm:reliable z is 80(m),instrinsic(f) is approximate si 4000(像素) 

   - **Update2D():** 更新2d box的大小。

     1. 向滤波器中添加测量值,进行滤波计算

        ```c++
            measurement << rect.width, rect.height;
            image_wh.AddMeasure(measurement);//一阶低通filter
            measurement << center.x, center.y;
            image_center.Correct(measurement);//Kalman_Filter
        ```

     2. 更新:得到该target最近最近检测物体(latest_object)的projected_box(单位：米)

        ![2d box update](apollo-track/20.png)

   8. 在Association之后通过IOU合并重复的targets(可能是不同相机得到的)

      ```c++
      CombineDuplicateTargets();
      ```

      - 对目前的targets_两两之间计算其各自含有的tarcked_objects之间的IOU以及box宽和高的差异，计算得到score (此处计算的tracked_object要求他们的时间戳之差小于0.05，同时来自不同传感器)

        ```c++
        score += common::CalculateIOUBBox(box1, box2); 
        ...
        score -= std::abs((rect1.width - rect2.width) *
                                      (rect1.height - rect2.height) /
                                      (rect1.width * rect1.height));
        ```

        将最终平均得分作为这两个`target`之间的相似程度，并将结果保存到score_list中(包括了两个target的索引及其得分)

      - 按照得分从大到小排序依次匹配，可以看出这一步骤与`OMTObstacleTracker::GenerateHypothesis()`相似不同的是我们这里要删除匹配成功的两个target中`id`大的那个,并将删除的那个target_del中的tracked_obj转移到target_save中。

        ```c++
            if (targets_[pair.target].id > targets_[pair.object].id) {
              index1 = pair.object;
              index2 = pair.target;
            }
            Target &target_save = targets_[index1];
            Target &target_del = targets_[index2];
            for (int i = 0; i < target_del.Size(); i++) {
              // no need to change track_id of all objects in target_del
              target_save.Add(target_del[i]);
            }
        ```

        并将target_save(target的引用)中的tracked_objects按照帧id(frame id)由小到大排序，并更新lastest_object。

        ```c++
            std::sort(
                target_save.tracked_objects.begin(), target_save.tracked_objects.end(),
                [](const TrackObjectPtr object1, const TrackObjectPtr object2) -> bool {
                  return object1->indicator.frame_id < object2->indicator.frame_id;
                });//将targe_save中的tracked_objects按照帧的id由小到大排序
            target_save.latest_object = target_save.get_object(-1);
        ```

        然后将target_del中的tracked_objects给清零，最后调用`ClearTargets()`即可将`targets_`多余的target清除掉(将target从后往前填空)

   9. 对经过滤波处理的box(单位：米)映射回图像坐标系（像素）
      

      ​	

## tracker_->Associate3D(frame):根据3D信息关联







## tracker_->Track(frame):跟踪算法