---

title: Apollo中多传感器标定与坐标系统
categories:
- autonomous
- apollo
tags:
- apollo
- calibration
mathjax: true
---

apollo中的坐标系统和多传感器标定及坐标转换(transform组件)

<!--more-->

## 1. 坐标系统

参考：`docs/specs/coordination_cn.md`

Apollo系统中采用的坐标系的定义：

### 全球地理坐标系

使用全球地理坐标系来表示高精地图(HD Map)中各个元素的地理位置。全球地理坐标系通常用途是用来表示纬度，经度和海拔。Apollo采用的是WGS84(World Geodetic System 1984)作为标准坐标系来表示物体的纬度和经度。通过使用该标准坐标系统，可以使用两个数字：x坐标，y坐标来唯一确定地球表面上除北极点之外的所有点，其中x坐标表示经度，y坐标表示纬度。全球地理坐标系的定义如图所示：

<img src="apollo-sensor_calibration\coordination_01.png" style="zoom:50%;" />

### **局部坐标系-东-北-天(East-North-Up ENU)**:

在Apollo中，局部坐标系的定义为：

z轴-指向上方（与重力线为同一直线）
y轴-指向北面
x轴-指向东面

<img src="apollo-sensor_calibration\coordination_02.png" style="zoom:50%;" />

ENU局部坐标系依赖于在地球表面上建立3D笛卡尔坐标系。

通用横轴墨卡托正形投影(Universal Transverse Mercator UTM),使用2D的笛卡尔坐标系来给出地球表面点的位置。该坐标系将地球划分为60个区域，每个区域表示为6度的经度带，并且每个区域上使用割线横轴墨卡托投影。在apollo系统中，UTM坐标系统在定位，规划等模块中作为局部坐标系使用。

关于UTM坐标系统的使用，遵从国际标准规范。开发者可以参考下述网站获取更多细节：

[http://geokov.com/education/utm.aspx](http://geokov.com/education/utm.aspx)

[https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system)

### 车辆坐标系-右-前-上(Right-Forward-Up RFU)

车辆坐标系的定义为：
z轴-通过车顶垂直于地面指向上方
y轴-在行驶方向上指向车辆前方
x轴-面向前方时，指向车辆右侧
车辆坐标系的原点在车辆后轮轴的中心

<img src="apollo-sensor_calibration\coordination_04.png" style="zoom: 50%;" />



### 传感器等硬件安装位置

下面的示例中，不同颜色箭头代表不同的坐标方向，实心代表垂直向上，包括了4个环视相机，3个radar，1个IMU，1个lidar。

<img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\apollo-sensor_calibration\calibration05.jpg" style="zoom: 33%;" />

首先拿到车辆的CAD模型，然后在软件环境中去放置sensor（尽量在软件平台中标注传感器的感知角度距离），需要考虑的不仅仅是空旷环境下传感器的感知范围，也要考虑到真实环境中的遮挡情况，同时要便于传感器融合（不同传感器的感知视野需要overlap）。需要注意的是传感器的安装位置的稳定性和安全性。

---

## 2. 多传感器标定

标定本质就是要获得不同传感器之间的相对位置，然后将各传感器的坐标系对齐，分为内参和外参，有些是传感器厂家提供，有一些则需要自己去重新标定：

- 内参：传感器自身性质，例如camera焦距，Lidar中各激光管的垂直朝向角。

- 外参：传感器之间的相对位置和朝向，用3自由度的旋转矩阵和3自由度的平移向量表示。

  > 内参虽然一般来说是固定的，但是随着长期的运行，也需要定期标定一下。

`Lidar`,`Camera`,`radar`之间有不同的观测角度和数据表现形式，要融合它们的各自的优点的前提就是需要它们之间相对的坐标变换关系,传感器的相对相对位置关系可以通过4*4变换矩阵M表示，实际有12个量需要计算表示旋转位移。
$$
M=
\begin{bmatrix}
R_{3\times3}& T_{3\times1}\\
0_{1\times3}&1
\end{bmatrix}
$$
但是实际上$R_{3\times3}$可以通过四元数表示转换为4个参数，实际上只需要标定7个参数。

<img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\apollo-sensor_calibration\174722.png" style="zoom: 33%;" />

### 2.1粗略标定

首先可以进行粗略的标定，在手凉传感器标定初值的时候，一定要分清楚是从哪一个源传感器到哪一个目标传感器，且这些数值是在目标传感器坐标系下，位移向量以米为单位，旋转以弧度为单位，如下所示，`frame_id`表示目标传感器，
`child_frame_id`表示源传感器，而`translation`和`rotation`是在`frame_id`即目标传感器坐标系下的short_camera的位姿

```yaml
header:
  stamp:
    secs: 0
    nsecs: 0
  seq: 0
  frame_id: Velodyne64 # Destination sensor
child_frame_id: short_camera # source sensor
transform:
  translation:
    x: 0.9356324324557267
    z: -0.5427650640523064
    y: -0.2485193364771597
  rotation:
    w: 0.5171207069875965
    x: -0.4789595241079692
    y: 0.5018481750475685
    z: -0.5013305874094867
```

### 2.2 录取数据

在录取数据之前先通过`Cyber_montior`工具来检查安装的传感器的状态，对应传感器的消息通道名称和帧率信息。绿色代表消息正常首发，红色代表消息没有出现。通过`Cyber Monitor`可以检查相应传感器的帧率是否正常，例如，激光雷达的帧率是10hz,相机的帧率15hz-20hz,GPS和惯导信号帧率为100hz。
还需要注意GPS的信号是否正常，质量是否足够高，如图红色框中，`Narrow_INT`是正常状态，此时三个方向上的标准差应该为1-2厘米级别。

<img src="apollo-sensor_calibration\gnss.png" style="zoom:75%;" />

同样，也可以通过`cyber_visualizer`命令来查看传感器的原始数据，保证数据质量，如图像是否模糊，lidar是否点云噪音过多，这种问题一般是由于硬件安装和EOS系统的不匹配造成的。

上述初步检查完成后，按照Apollo的建议采集有效的传感器标定数据，脚本文件`($Apollo)/scripts/record_bag.py`

对于激光雷达，只需采集原始点云数据；对于相机，建议采集未压缩的原始图像信息；对于惯导和GPS需要采集他们的odometry信息。右图的三个红框分别展示了典型的图像信息、点云信息以及GPS和惯导信息。在使用Apollo DreamView进行数据采集前，可以通过上述步骤修改对应脚本中的消息列表，以适配自己的传感器方案,例如lidar到gnss的标定不需要camera数据，则可以过滤图像数据，以减小数据包的大小。

<img src="apollo-sensor_calibration\calibration06.jpg" style="zoom: 33%;" />

gnss/odometry:100Hz通道的数据组成：

<img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\apollo-sensor_calibration\odometry.png" style="zoom: 67%;" />

### 2.3 标定策略

- **相机内参标定：**

- **Lidar内参标定：**

  

- **Lidar-to-GPS外参标定：**
  在空旷的地方绕8字，每一个时间点GPS都会得到车辆的位置信息(原点是GPS安装点)，然后把每一帧点云都投影到GPS坐标原点的坐标系中，将点云拼接到一起解一个优化，在采集是，希望场景中没有过多的动态障碍物，地面平整，场景中有类似数目，电线杆之类的静态障碍物。由于组合惯导和Lidar没有直接的测量数据对应，所以采用**手眼标定**方法获得外参初值，然后采用基于点云拼接质量进行外参优化。
  具体标定方法参考：https://github.com/ApolloAuto/apollo/blob/master/docs/specs/lidar_calibration.pdf

  <img src="apollo-sensor_calibration\230426.png" style="zoom: 50%;" />

  

  <img src="apollo-sensor_calibration\cal06.jpg" style="zoom: 33%;" />

- **Lidar-to-Lidar外参标定：**
  每个Lidar与Rigel之间独立进行ICP，然后得到相对`Rigel`的位移旋转矩阵，再通过传递得到Lidar之间的相对位置。验证方法是将所有的点云都投影到真实的物理世界中，若边缘清晰则标定正常。

  ![](apollo-sensor_calibration\205819.png)

- **Lidar-to-Camera外参标定：**
  在标定间中，贴满二维码(April-Tag 具有高鲁棒性和识别精度的二维码)，每个二维码对应的ID都不一样，然后在标定间中间放置毫米级精度激光扫描仪`Rigel`,对标定间进行3D建模，这样就知道了标定间墙面上任意点的3D位置(包括了April-Tag的四个角点)。由于`Lidar`与`Rigel`都是点云数据，两个点云之间做ICP，得到对应的位移旋转矩阵，相机通过`Rigel`也可以得到对应的位移旋转矩阵，然后两个位移旋转矩阵进行传递即可得到相对的位置。验证方法是将3D点投影到图像中，观察边界是否一致。
  ![](apollo-sensor_calibration\205718.png)

- **Camera-to-Camera外参标定**

  在标定间中，长焦和短焦的相机都能够过的Tag的四个角点3D坐标，同时在相机中对应的2D坐标也已知，2D坐标 3D坐标都有了，那么通过多个点对求解这个`PNP`问题，可以得到短焦相机和长焦相机相对Rigel各自的位移旋转矩阵，最后就可以得到它们之间的相对坐标。标定完成后，需要验证标定结果，对于图像来说可以通过可视化，比如长焦的图像范围一般是包含在短焦的视野范围内的，那么将长焦图像投影到短焦图像上没有重影比较自然则应该比较标准。
  <img src="apollo-sensor_calibration\204947.png" style="zoom:50%;" />

- **自然场景中的Lidar-to-Camera外参标定**

  自然场景下的标定有一些要求，需要在场景中找到一些边缘(比如路边的标识牌)，针对lidar(边缘处lidar的深度会有变化)和camera各自对边缘的坐标作为考量的指标，将两个sensor对齐。
  相机到lidar的标定，通过4个5的方法来指导开车数据采集过程，即在一个较为空旷，拥有大量静态障碍物的场景中，无人车以大约5英里每小时的速度直线行驶5米后刹车，之后完全停止大约5秒，重复以上步骤5-6次。

  完成数据采集后，可以通过命令`cyber_recorder info ${bagname}`来检查采集的数据包。通过计算每秒钟采集的数据的帧率，可以大致了解数据是否完整的被存放，例如，Gps惯导信息，帧率为100HZ,在4秒左右的数据包中有大约430个数据，符合预期。而红框中所示的相机，其消息数为0，如果我们需要涉及到相机的标定，那么这个数据集的采集是不成功的。
  ![](C:\Users\jia_z\Desktop\blogrepo\source\_posts\apollo-sensor_calibration\recorder.png)

- **自然场景中的Bifocal Camreas外参标定**
  自然场景下，视野有重叠的多个相机之间的对齐，同样在重叠的视野中找到一个边缘比较锐利的物体，进行外参标定。

- **Camera-to-Radar外参标定**
  由于radar不知道高度，其返回的是一个平面内的距离角度信息，需要得到radar相对相机的高度。一般是假设radar是水平安装，安装位置已在汽车的cad模型中固定好了，然后通过CAD模型的测量可以得到这个高度。但是相机的pitch角度即相对radar的俯仰角度(假设radar是水平的)是需要标定的。在假设radar水平的前提下，标定问题退化为相机与地面（水平面）的pitch角度问题，这样就可以借助Lidar,通过Lidar可以得到地面水平面，Camera相对于Lidar已经标定好了，则Camera相对于地面的pitch角度就可以测量出来，这个角度就等同于Camera相对radar的水平角度。

### 2.4 通过Apollo工具提供有效数据

<img src="apollo-sensor_calibration\calibration10.jpg" style="zoom: 33%;" />

当完成数据采集后，可以通过Apollo平台中数据分析提取工具，提取有用的标定信息，仅需一行命令，就可以自动完成数据的提取和压缩，可以将原50GB的数据压缩到3-5GB（5lidars/5Cameras/gps calibration needs）

```bash
python /apollo/modules/tools/sensor_calibration/extract_data.py --config /apollo/modules/tools/sensor_calibration/config/camera_to_lidar_calibration.config
```

`*.config`配置文件中分为三个部分：第一个部分是io配置，主要是指定标定任务的名称，以及相应的输出路径；第二个部分，主要为指定输入的数据包路径，可以是完整的数据包路径列表，也可以是一个包含多个数据包的文件夹路径。最后一部分主要是指定所需提取的消息名称以及采样率。对于小容量信息(比如GNSS里程计信息)，建议完整保留，采样率为1.

<img src="apollo-sensor_calibration\config.png" style="zoom:40%;" />

### 2.5 通过Apollo产生准确的标定参数

对于相机内参，由于需要采集棋盘格图像，要求棋盘格的成像能够覆盖图像的各个区域，并且图片中的棋盘格完整。采集步骤如下：
将设备固定在三脚架上，开启相机，观察标定板在相机上的成像。将图像分为图片中的五个区域，将标定板正对相机并移动标定板将成像置于这5个区域内，在每个区域中，分别绕标定板的X,Y轴旋转正负30度左右各一次，绕z轴旋转正负45度左右各一次，并采集图像。

<img src="apollo-sensor_calibration\caliop.png" style="zoom: 33%;" />

采集完相应的棋盘格图像后，可以使用OpenCV自带的摄像机内参标定工具完成标定。完成内参标定后，按照Apollo建议内参格式，生成对应的内参文件。

<img src="apollo-sensor_calibration\accurate.png" style="zoom:33%;" />

当完成上述步骤后，可以将数据提取工具产生的压缩包以及摄像头的内参文件，发给 Apollo 服务团队，完成传感器的标定。https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/multiple_lidar_gnss_calibration_guide.md

https://login.bce.baidu.com/?redirect=http%3A%2F%2Fconsole.bce.baidu.com%2Fapollo%2Fcalibrator%2Findex%2Flist

<img src="C:\Users\jia_z\Desktop\blogrepo\source\_posts\apollo-sensor_calibration\103354.png" style="zoom: 50%;" />

### 多传感器数据融合 关系

## 3. Transform Wrapper

整个Transform部分的封装格式和相互关联应该为

> `third_party/tf2(ros) -> modules/transform -> perception/onboard/transform_wrapper`

此部分详细参考文件`apollo-transfom_wrapper.md`

## 4. Apollo提供的标定工具

参考文件：docs/quickstart/multiple_lidar_gnss_calibration_guide_cn.md

参考文件：https://github.com/ApolloAuto/apollo/blob/master/docs/specs/apollo_lidar_imu_calibration_guide.md



## 参考资料

https://blog.csdn.net/qq_41204464/article/details/102945454

https://gitbook.cn/books/5cd8e067d7a0dd4c0f47391a/index.html

https://bit.baidu.com/productsBuy?id=81&chapterId=224