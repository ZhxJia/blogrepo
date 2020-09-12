---
title: 《Deep Learning for Image and Point Clound Fusion in Autonomous Driving:A Review》
tags:
- sensor fusion
categories:
- paper reading
- sensor fusion
mathjax: true
---

​	相机与Lidar点云融合的综述文章。针对点云和图像数据结构特征的差异，提出了目前这两种传感器融合方面的切入点和发展趋势。无人驾驶要求对周围3D环境进行可靠的感知，单目相机原理上不能提供3D几何信息，而双目相机虽然可以提供3D几何信息，但其引入计算量同时探测距离有限对环境条件要求较高，因此在无人驾驶环境下使用的较少。Lidar能够提供高精度的几何信息同时不受光线条件的影响，但是存在采样稀疏，语义信息匮乏的问题。
​	为了应对无人驾驶感知任务的挑战，当前许多研究致力于如何有效的融合这两种互补的传感器信息。

<img src="pr-review_Image_PointCloud_Fusion\image-20200521161943634.png" alt="image-20200521161943634" style="zoom: 33%;" />

<!--more-->

深度学习方法在图像中的应用：图像中编码的是可见光信息，目前基于CNN的相关网络结构得到的广泛的研究。
由于点云数据结构(无序，不规则,密度不均衡(近多远少))的特点，直接应用已有的网络结构比较困难，目前针对点云数据的处理方式有：

1. Volumetric representation based(基于体素网格的方法)：将点云划分到3D网格中，从而可以直接应用3D convolution
   这种划分方法存在问题：高分辨率的3D体素网格(voxel-space)大幅增加了计算量，同时点云的稀疏性导致了大量的空网格且会随着分辨率的提升立方增长。
   [参考:Voxelnet](https://arxiv.org/pdf/1711.06396.pdf)    [参考:Voxnet](https://ieeexplore.ieee.org/document/7353481)
2. Tree-like representation based(基于树状结构的方法)：出发点为了减轻高分辨率带来的高计算量的约束，通过将点云划分到一系列的不平衡树（八叉树），使得低密度的点只有较低的分辨率，减少不必要的空间和计算资源浪费。特征提取采用类卷积的操作。
   [参考:Octnet](https://arxiv.org/abs/1611.05009)     [参考:context-based octree](https://ieeexplore.ieee.org/document/8784388)   [参考:octree guided cnn](https://arxiv.org/abs/1903.00343)    [参考:3dcontextnet](https://arxiv.org/abs/1711.11379)
3. Point representation based: (基于原始点云的表示方法)：该方法采用原始点云，开创工作是[pointNet](https://arxiv.org/abs/1706.02413)
4. 2D view representation based(基于点云投影2D视图的表示方法)：这种方法将点云投影到某个2D平面,2D平面中每个点编码了点云的特征(类似于图像的特征图)，常用的2D投影平面例如鸟瞰图(bird's-eye view,BEW),这种投影方式视角遮挡最小，同时原始的物体朝向和x,y坐标被保留。目前常用的特征表述包括了(average points height,density and intesity)。标准的2D卷积和现有的卷积框架结构可以被直接使用。
5. Geometric representation based(基于几何的表示方法)：点云可以表示为图(拓扑)结构，类卷积操作可以实现用于特征提取。
   [参考:graph topology inference](https://arxiv.org/abs/1905.04571)   [参考:deep cnn for graph data](https://arxiv.org/abs/1506.05163)   [参考:edge-conditioned filters](https://arxiv.org/abs/1704.02901)

点云的特征表示方式目前还是一个开放的问题。

## 1. 论文总体结构

论文从四个方面(Depth completion , object detection, semantic segmentation, object tracking)概述了图像和点云融合的相关进展。

![image-20200521161607067](pr-review_Image_PointCloud_Fusion\image-20200521161607067.png)



## 2. 无人驾驶感知的挑战

### 2.1 Depth Completion

深度补全是指通过上采样稀疏不规则的数据到密集规则的数据。Camera-Lidar融合的方法通常利用高分辨率的图像指导上采样，实现pixel-wise层级上的深度图。下图按照时间顺序给出了深度补全模型和对应的融合层级。

![image-20200524204555123](pr-review_Image_PointCloud_Fusion\image-20200524204555123.png)

### 2.2  3D object detection

​	3D目标检测方法的目的是在3D空间中定位，分类以及估计障碍物的朝向。目前有两种主流的实现方法：
​	two-step(Sequential)和one-step。two-step(Sequential)包含了两个阶段：proposal和regression。one-step直接并行输出2D和3D信息。

下图按照时间顺序展示了3D目标检测的相关工作，并标注了相机和lidar对应的融合层级。

![image-20200526193634386](pr-review_Image_PointCloud_Fusion\image-20200526193634386.png)

1. 2D proposal based sequential models
   试图利用2D图像语义信息提供3D proposal的初始区域：利用现有的图像处理框架生成2D region proposal，然后投影到3D空间，2D->3D的投影过程有两种主要方法，其中之一是将图像平面边界框投影到3D点云空间，结果形成了一个视锥型搜索空间，第二种方法是将点云投影到图像平面，结果是使点云具有2D语义信息。
   下面按照融合层级给出这方面的相关工作：
   (1) Result-level Fusion:信息融合发生在最后result阶段
   	这种方法的直接直觉来源是利用现有的2D目标检测方法缩小3D目标检测的搜索区域。最直接的方法就是将2D bbox反投影到3D空间，这样将多个感兴趣的小区域替代整个点云的处理以减少计算量，然而这种方法由于采用了2D bbox的检测结果因此整体性能会受限于2D detector的性能。Result-level Fusion核心思想是不利用多模态的数据进行互补，只是用于减少计算量。
   	早期的工作有 [F-PointNet](https://arxiv.org/abs/1711.08488)将图像生成的2d bbox反投影得到的视锥体区域点云送入PointNet中。[A general pipeline
   for 3d detection of vehicles](https://arxiv.org/abs/1803.00387)这篇论文中在2D到3D proposal生成阶段加入了额外的基于模型拟合方法的细化操作，将感兴趣区域内的背景点去除，将过滤后的点送入bbox回归网络中。[RoarNet](https://arxiv.org/abs/1811.03818)将细化操作用神经网络代替。
   这些方法存在的问题是：假设得到的seed region仅包含一个目标，这对于拥挤场景和小目标(如行人)是不合理的。
   	该问题的解决方法有将2D目标检测器替换为2D语义分割网络,这样region-wise级别的提议改为point-wise级别的提议。[Intensive Point-based Object Detector :IPod](https://arxiv.org/abs/1812.05276) 在这个方向上做了相关工作，首先将点云投影到图像中利用2D语义分割信息过滤背景点。

   (2)Multi-level Fusion:结合result-level和feature-level。
   	典型的工作有[PointFusion](https://arxiv.org/abs/1711.10871),利用现有2D目标检测器生成2Dbbox，通过将点云投影到图像平面中选择对应在bbox中的点，最终分别通过Resnet和Pointnet基础架构在每一个proposal上融合图像和点云特征 预测3D目标，然而在proposal阶段仅仅利用了图像这一个模态。[SIFRNet](https://arxiv.org/abs/1901.02237)中frustum proposal首先从图像中生成，在frustum proposal中的点云特征与对应的图像特征融合用于最终3d bbox的回归。IPod

   (3)feature-level Fusion:多模态融合
   	多模态融合的早期尝试是在像素层级pixel-wise上进行的，将3D几何特征通过某些方式(例如鸟瞰图)转换为2D图像格式，然后附加到图像通道上，基本理想是将3D几何特征投影到图像平面上然后利用现有的图像处理框架直接处理，这种方法的输出也是在限制于图像平面中，不是3D空间物体检测的理想方式。[DepthRCNN](https://arxiv.org/abs/1407.5736)编码3D几何信息(HHA，来自RGBD)到相机RGB通道中。[Fusing lidar and images for pedestrian detection using convolutional neural networks](https://ieeexplore.ieee.org/document/7487370)这篇论文中进行了扩展，3D几何信息(HHA)由lidar产生。
   	为了在3D空间中更准的定位物体，目前常用point-wise层级上的融合，通过这种方式图像与点云中的点一一关联。[PointPainting](https://arxiv.org/abs/1911.10150)将点云投影到2D语义分割图中(利用之前提到过的Ipod)，然而除了利用2D语义过滤点云，2D语义仅简单地附加到点云中作为额外的通道。

2. 3D proposal based sequential model

   ​	相较于2D proposal,该方法直接从2D/3D数据生成3D Proposal。消除2D到3D转换的阶段大大缩小了目标检测的3D搜索空间。通用的生成3D proposal的方法有多视角(multi-view)或者点云体素化(voxelization)

    - 多视角方法(multi-view)利用点云的鸟瞰图(bird'eye view,BEV)进行3D proposal生成。鸟瞰图保留物体的旋转方向和x,y坐标信息。
   - 点云体素化(voxelization)将原先连续的不规则的数据结构转换为离散的规则的数据结构，这样可以直接利用3D
   CNN,缺点是损失了空间分辨率。

   ​	(1) Feature-level fusion:
   ​		从BEV表示生成3D Proposal的重要工作是[MV3D](https://arxiv.org/abs/1611.07759),利用lidar特征图(包括height,density,intensity)生成3D候选区域，然后将他们投影到前视图(front view)与图像数据融合进行边界框回归。融合发生在Roi pooling阶段，因此是ROI-level层级的融合，这项工作中存在的问题有通过BEV生成3D proposal过程中假设所有的目标都被捕获，针对小目标物体(行人，骑自行车的人)的识别可能表现并不是很好，因为可能被其他大的目标遮挡。其次，小目标在连续的卷积下采样过程中可能丢失。同时，在roipooling过程中进行融合可能会损失细粒度信息。
   ​		为了提升对小目标的检测能力，[Aggregate View Object Detection network (AVOD)](https://arxiv.org/abs/1712.02294)通过同时利用BEV和图像特征提升MV3D中的proposal阶段。图像分支利用auto-encoder结构上采样最终特征图到它原始大小。[Scanet](http://150.162.46.34:8080/icassp2019/ICASSP2019/pdfs/0001992.pdf)中也实现了encoder-decoder并利用了Spatial-Channel Attention(SCA)和Extension Spatial Unsample(ESU)模块。

   ​		上述方法以object为中心在ROI-pooling阶段(roi-wise fusion)进行融合损失了细粒度的几何信息。[ContFuse](http://openaccess.thecvf.com/content_ECCV_2018/papers/Ming_Liang_Deep_Continuous_Fusion_ECCV_2018_paper.pdf)通过在point-wise进行融合应对这个问题，这种point-wise融合通过[continuous convolutions](http://openaccess.thecvf.com/content_cvpr_2018/papers/Wang_Deep_Parametric_Continuous_CVPR_2018_paper.pdf) 实现。首先提取BEV视图中每一个像素的K个最近邻点，然后将这些点映射到图像平面中以检索对应的图像特征，最终融合的特征向量根据与期望像素的几何偏移加权送入MLP中。 然而这种融合方法由于点云的稀疏性可能不能充分融合高分辨率图像的特征。[Multi-task multisensor fusion for 3d object detection](http://www.cs.toronto.edu/~byang/papers/mmf.pdf)中综合了多个层级的融合策略(signal-level,feature-level,multi-view,depth completion)。上述基于point-wise/pixel-wise层级融合会产生特征模糊(feature blurring)的问题,即可能存在点云中的点对应多个图像中的像素或者反之，这会混淆数据融合。

   ​		直接将RGB图像信息附加作为voxel的额外通道是最简单的组合体素点云和图像的方法，这种方法存在的问题是体素化表示破坏了细粒度局部几何信息，同时由于图像和点云分辨率不一致是的融合过程效率较低。

3. One-step Models
   一步模型将proposal generation和bbox回归的过程合为一步。[MVX-Net](https://arxiv.org/abs/1904.01649)介绍了两种方法用于在point-wise和voxel-wise层级融合图像和点云，采用预训练的2D CNN用于图像特征提取，同时采用[VoxelNet](https://arxiv.org/abs/1711.06396)作为框架从融合的点云中预测物体。在point-wise融合方法中，点云首先被映射到图像特征空间用于在VoxelNet网络处理前提取图像特征。在Voxel-wise融合方法中，在将非空的voxel映射到图像特征空间之前首先体素化点云。
   [Lasernet](https://arxiv.org/abs/1903.08701)是一个多任务和多模态网络，用于融合点云和图像的3D目标检测和3D语义分割。两个CNN并行处理lidar产生的深度图像和前视图并且通过将点投影到图像平面关联对应的图像特征。特征图送入LaserNet中预测每一个点bbox的分布并生成最终的3D proposal。

### 2.3 2D/3D semantic segmentation

回顾camera-lidar融合的2D/3D语义分割，实例分割方法，下图按时间顺序给出3D语义分割网络和对应的融合等级。
<img src="pr-review_Image_PointCloud_Fusion\image-20200527211057103.png" alt="image-20200527211057103" style="zoom:67%;" />

1. 2D Semantic Segmentation
   (1) Feature-level Fusion:
   	[Self-supervised model adaptation for multimodal semantic segmentation](https://arxiv.org/abs/1808.03833)采用不同深度多阶段特征融合进行语义分割。[Lidarcamera fusion for road detection using fully convolutional neural networks](https://arxiv.org/abs/1809.07941)论文中通过利用上采样的深度图和图像来进行语义分割，通过上采样来自点云的稀疏深度图同时利用相机图像获得密集的深度图。

2. 3D Semantic Segmentation
   (1) Feature-level Fusion:

   ​	[3DMV](https://arxiv.org/abs/1803.10409)是融合图像语义和点云(RGBD)特征的多视角网络，来自多个视角的图像特对齐后通过2D卷积网络提取特征并映射到3D空间。这种基于体素的方法性能受限于体素的分辨率。针对该问题，[UPF](https://arxiv.org/abs/1908.00478)是基于点的3D语义分割框架，基于根本方法是[PointNet++](https://arxiv.org/abs/1706.02413)

3. Instance Segmentation

   略

>  上述各种方法，目前在自动驾驶的实际场景中应用较少，没有细看，如果以后用到，可直接参考论文

### 2.4 3D Object Tracking

多目标跟踪是自动驾驶感知不可缺少的环节，针对camera-lidar融合的目标跟踪方法总结如下，性能评价指标参考[KITTI](http://www.cvlibs.net/publications/Geiger2012CVPR.pdf)

![image-20200527220324567](pr-review_Image_PointCloud_Fusion\image-20200527220324567.png)

1. Detection-Based Tracking(DBT)基于检测器的目标跟踪方法

   DBT(Tracking-by-Detection)跟踪框架包含两个阶段：第一步通过检测器检测目标，第二步进行数据关联，将同一目标按照时间轴构成一条跟踪轨迹。[End-to-end learning of multi-sensor 3d tracking by detection](https://arxiv.org/abs/1806.11534)提出了端到端的DBT训练框架，为了实现端到端的学习，检测和匹配通过一个深度结构化模块(DSM)实现。
   貌似都是离线跟踪的方法，目前暂时用不到，详细看论文。

2. Detection-Free Tracking(DFT)
   [Complexer-YOLO](https://arxiv.org/abs/1904.07537)是一个解耦3D目标检测和跟踪的框架。

## 3. 未来研究方向


感知模块是无人驾驶中下游决策，规划，控制的前提，它的性能和可靠性是整个无人驾驶系统性能优劣的首要因素。因此，利用相机和lidar融合以更好的理解复杂环境。下表给出了提升camera-lidar融合算法的性能，可靠性以及相关工程实践的一些挑战。

![image-20200527223109666](pr-review_Image_PointCloud_Fusion\image-20200527223109666.png)

融合的发展趋势有：

- 2D to 3D:

- sing-task ot multi-tasks:考虑到移动平台的计算性能和实时性需求，网络往往需要多任务输出。

- singal-level to multi-level Fusion:早期的工作集中于输入信号层级的融合，例如将点云投影到图像平面等，最近很多模型视图融合在多个层级上。

  



深度估计：单目深度估计，融合深度估计

故障检测：相机在不利环境下(例如重影，光线不足，)



[自动驾驶相机-激光雷达深度融合综述及展望](https://mp.weixin.qq.com/s?__biz=MzI1ODYwOTkzNg==&mid=2247500603&idx=1&sn=c8df20167dffe5ba6c4e850555293ad4&chksm=ea070544dd708c5270a1380f95ff52386204a96d1c2b026697ded4e7f9089d6fc114ff62a026&mpshare=1&scene=1&srcid=&sharer_sharetime=1586875366394&sharer_shareid=251c7f638257dafd65dd3f515ad533cb&key=0b6318e4d66e8464fd862767da5f8c51c286edb23c13d2df9762e0649316e1f8d9a5a96d352ad1c430a4868e0b78a343a61c26068054087211927f5b411c6fb5db9b716f1c218582b7d9d8347625c14b&ascene=1&uin=MjU0MTcyMzYzNw%3D%3D&devicetype=Windows+10+x64&version=62090070&lang=zh_CN&exportkey=A9Nk8Y3UMCPDtCIcH%2B6a0Lg%3D&pass_ticket=M9LuIPgA%2BNn%2FM1hrwO9oOAOPG2geFBvwMZ15XwJ5RivYEn75h8Qufft2SOni6CDL)

