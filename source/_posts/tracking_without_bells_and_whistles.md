---
title: tracking without bells and whistles论文阅读
categories:
- 论文
tags:
- track
mathjax: true
---

tracking without bells and whistles 论文阅读笔记
**A detector is all you need for Multi-Object Tracking**

<!--more-->

## FasterRCNN回顾

![](tracking_without_bells_and_whistles\20170324121024882.png)

FasterRCNN将**特征提取、Proposal提取、BoundingBox回归、分类**整合到一个网络中：

- **特征提取**：Faster R-CNN首先使用基础的conv+relu+pooling层提取候选图像的**共享特征图**，该特征图被共享用于后续的RPN(Region Proposal Network)层和全连接层(FC)
- **区域候选网络**（Region Proposal Network）：RPN网络用于生成候选图像块。该层通过`softmax`判断锚点`anchors`属于前景(foreground)或者背景(background)的概率，再利用边界框回归修正`anchors`获得获得较精确的proposals。(上面的分支分类anchor的前景和背景，下面的分支计算前景（即目标）anchor的偏移量(bbox回归)，实际上RPN实现了目标的定位)
- **目标区池化**(Roi Pooling):该层收集`输入特征图`和`候选的目标区域`，综合这些信息提取`目标区域的特征图`(proposal feature)，送入后续的全连接层进行目标类别的判断
- **目标分类**：利用`目标区域的特征图`计算目标区域的类别，同时再次边界框回归获得检测框最终的精确位置。

FasterRCNN特征图每个点有9个anchor,而RPN实际就是在原图像的尺度上，设置了密密麻麻的anchor，然后通过CNN判断哪个anchor是没有目标的背景(background)，哪些anchor是有目标的前景(foreground)，boundingbox的回归用到了两次，实际训练过程如下，分为两次循环：

1. 在已经训练好的model上，训练RPN网络，对应stage1_rpn_train.pt (RPN出的bbox回归)
2. 利用步骤1中训练好的RPN网络，收集proposals，对应rpn_test.pt
3. 第一次训练Fast RCNN网络，对应stage1_fast_rcnn_train.pt(最终检测处的bbox回归)
4. 第二训练RPN网络，对应stage2_rpn_train.pt
5. 再次利用步骤4中训练好的RPN网络，收集proposals，对应rpn_test.pt
6. 第二次训练Fast RCNN网络，对应stage2_fast_rcnn_train.pt

<img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\tracking_without_bells_and_whistles\23.jpg" style="zoom:80%;" />

<img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\tracking_without_bells_and_whistles\66.jpg" style="zoom:30%;" />

> 参考链接：
>
> https://blog.csdn.net/shenziheng1/article/details/82907663
>
> https://zhuanlan.zhihu.com/p/32404424

## tracking without bells and whistles

### 网络结构流程

![](C:\Users\jia_z\Desktop\blogrepo\source\_posts\tracking_without_bells_and_whistles\tracking.png)

跟踪器：
	轨迹(trajectory)由一系列针对k目标的有序bounding box组成的$T_k=\{b_{t1}^k,b_{t2}^k,...\}$, $b^k_t=(x,y,w,h)$，$t$代表当前帧。
	同时定义第$t$帧中的目标边界框集合为$B_t=\{b_{t}^{k1},b_t^{k2},...\}$,注意每个跟踪轨迹$T_k$或者边界框集合$B_t$包含的元素可以少于序列中轨迹或帧的总数量。

1. **初始化**
   在t=0时，跟踪器初始化为第一次检测器检测到的目标的集合$D_0=\{d_0^1,d_0^2,...\}=B_0$,在上图中，演示了接下来将要进行的两个步骤，对于给定帧$t$的**bounding box regression** 和 **track initialization**
2. **Bounding box regression**













