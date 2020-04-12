---
title: Multi-View 3D Object Detection Network for Autonomous Driving
tags:
- 目标跟踪
categories:
- 论文阅读
mathjax: true
---

论文阅读：自动驾驶场景下的3D障碍物检测网络

<!--more-->

### 3D 目标检测回顾

- 点云3D目标检测
  **voxel grid representation**
  	3D convolution: <3D Fully Convolutional Network for Vehicle Detection in Point Cloud>
  	3D卷积的计算代价很大

  **multi-view representation**
  	<Volumetric and multi-view cnns for object classification on 3d data.>

- 图像3D目标检测
       <Monocular 3d object detection for autonomous driving.> 从单目图像中生成3D proposal
       还有结合运动和地平面结构信息，将2D Box迁移到3D Box
       \<A continuous occlusion model for road scene understanding.>
       <Joint sfm and detection cues for monocular 3d localization in road scenes.>
  基于图像的方法通常依赖于准确的深度估计和标志物检测。

- 多模态融合的3D目标检测
      <Onboard object detection: Multicue, multimodal, and multiview random forest of local experts>综合了图像，深度，光流进行2D行人检测。

目前使用多种数据融合进行检测的文章还不是很多，MV3D受分型网络FractalNet和Deeply-Fused Net网络启发，其中
FractalNet迭代重复基础模型构造网络，Deeply-Fused Net类似通过结合深、浅基网络实现基网络之间的深度融合。

- 3D object proposal
  类似于2D object proposal ,3D目标提议方法生成 包含大多数目标的3D候选框子集，由于3D卷积计算代价比较大，文中使用的方法是在鸟瞰图(bird view)上使用2D卷积生成正确的3D Proposal.

### MV3D net

MV3D采用了3D点云图的多视角特征和2D图像作为输入。首先根据点云bird view生成了3D object proposal，然后由此根据提议在局部区域上对多视角特征进行深度融合。融合的特征用于分类和oriented 3D box回归。 

- 点云的表示形式
  ![image-20200412211428658](pr_MV3D\image-20200412211428658.png)

  其中bird view 通过点云的高度，密度，强度表示。以0.1m为分辨率建立离散化网格，每个网格的高度由该网格中最高的点表示，为了编码更多的高度特征，点云被平均分成了M片。强度特征为对应高度图中点的反射强度，密度特征为每个cell中点的个数，为了归一化该特征，通过$min(1.0,\frac{log(N+1)}{log(64)})$ 其中N为该Cell中点的个数。整个bird view产生的特征通道的个数为(M+2)个。
  front view 相当于Bird view提供了互补的特征，因为Lidar点云非常稀疏，将其投影到图像平面将会得到稀疏的2D点，因此，文中将点映射到了柱面（而不是图像平面）以得到一个相对密集的2d点云图。给定一个3d点$p=(x,y,z)$ 在前视图中的坐标为$p_{fv}=(r,c)$ 通过下列方式计算：
  (参考论文Vehicle Detection from 3D Lidar Using Fully Convolutional Network)
  $$
  c=\lfloor atan2(y,x)/\Delta\theta\rfloor \\
  r = \lfloor atan2(z,\sqrt{x^2+y^2}/\Delta\phi \rfloor)
  $$
  其中$\Delta\theta$和$\Delta\phi$分别是水平和垂直方向laser分辨率，将前视图编码为三通道特征图。

- 3D proposal Network
  通过鸟瞰图来进行3D proposal有以下几个优点：
  保留了原始的物体尺寸，其次鸟瞰图中的物体占据了不同的空间，因此避免了相互之间的遮挡问题，最后，在道路场景中由于物体基本都位于地平面上，因此在垂直方向上位移很小，因此鸟瞰图对于获取准确的3D object proposal至关重要并且是可行的。
  给定鸟瞰图，网络生成3D box proposal从一组3D先验框（anchor）中,每一个box通过$(x,y,z,l,w,h)$进行参数化（雷达坐标系下的中心和尺寸(m)）,对于每一个3D先验框，相对应的鸟瞰图的anchor$(x_{bv},y_{bv},l_{bv},w_{bv})$ 通过离散化
  $(x,y,l,w)$来获取，通过在训练集的object size上进行聚类，设计了N个3D 先验框。在车辆检测中，先验框$（l,w）$大小设置为$\{(3.9,1.6),(1.0,0.6)\}$ 高度设置为1.56m,通过旋转鸟瞰图90°可以得到N=4个先验框。
  注意，在proposal生成阶段，不进行物体朝向的判断。

  由于离散化过程中分辨率设置为0.1m，则物体的边界框在鸟瞰图中可能仅有5~40个点，对于极小目标的检测是比较困难的，采用该[文章][https://github.com/zhaoweicai/mscnn]中的方法，在最后proposal layer的最后卷积层中进行2x 上采样。

  proposal 回归$t=(\Delta x,\Delta y,\Delta z,\Delta l,\Delta w,\Delta h)$ ,这一部分与RPN类似，$（\Delta x,\Delta y,\Delta z）$是由anchor尺寸归一化的值，而$(\Delta l,\Delta w,\Delta h)$则是通过$log\frac{gt(s)}{anchor(s)},s\in(l,w,h)$计算，损失函数计算：分类采用类别交叉熵，位置采用$smooth \ l_1$ 
  训练过程中，通过计算anchor与ground truth的IOU来确定anchor的类别标签(大于0.7认为positive，小于0.5认为是negative,中间的被忽略，这部分基本与RPN一致)。由于点云是稀疏的，因此存在大量的空anchor，在训练和测试过程中去除，以减少计算量。

  最后采用NMS(0.7)去除冗余，前2000个boxes在训练过程中被保持，在测试时，仅保留300个boxes

- Region-based Fusion Network

  通过设计region-based的融合网络来综合不同视角的特征图，并且同时进行物体分类和3D box方向的回归。

  - Multi-View ROI pooling
    由于来自不同视角的特征图具有不同的分辨率，采用ROI Pooling 去获得相同长度的特征向量。将生成的proposal映射到3D空间中的各个视图中，在此处，映射到三个视图（鸟瞰图BV，前视图FV，相机图像RGB），通过给定一个3D proposal ，我们可以得到ROIs:
    $$
    ROI_v=T_{3D->v},v\in\{BV,FV,RGB\}
    $$
    T表示从雷达坐标系转换为三个视图的坐标系，最终得到固定长度的特征向量。

  - Deep Fusion
    提出了一种分层融合多视图特征图的Deep Fusion方法：

    <img src="pr_MV3D\image-20200412232728365.png" alt="image-20200412232728365" style="zoom:67%;" />

    输入为三个视图的特征图，上图展示了不同的融合策略。

  - Oriented 3D box regerssion

    给定了多个视角特征的融合网络，然后由此根据3D proposal 回归 3Dbox的朝向，回归包括了3D框的8个三维角点坐标，总共24个向量，尽管这对于标记方向是冗余的，但是作者发现这种编码方式比通过中心点和尺寸的编码方式效果好。物体的朝向可以通过角点计算出来。

    通过multi-task 损失函数同时训练类别和物体的朝向，这部分与proposal的损失函数一致。

  - Network Regularization（网络正则化）

    这部分参考了FractalNet,采用两种方法调整网络：`drop-path training`和`auxiliary losses`。每一次迭代随机选择 global drop-path，如果一个global drop-path被选择，则从三个视图中随机选择一个视图。如果local drop-path被选择，则以0.5几率随机抛弃输入到join的节点，但要保证每个join（连接点）至少有一个输入。

    ![image-20200412234940402](pr_MV3D\image-20200412234940402.png)

    在推断过程中，上述附加网络将会去除。







代码实现：https://github.com/bostondiditeam/MV3D
				  https://github.com/leeyevi/MV3D_TF

论文解读：https://zhuanlan.zhihu.com/p/86312623