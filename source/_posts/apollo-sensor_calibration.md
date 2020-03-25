---

title: Apollo中多传感器标定与坐标系统
categories:
- apollo
- perception
tags:
- 传感器标定
mathjax: true
---

apollo中的坐标系统和多传感器标定及坐标转换(transform组件)

<!--more-->

## 1. 坐标系统

参考：`docs/specs/coordination_cn.md`

Apollo系统中采用的坐标系的定义：

### 全球地理坐标系

使用全球地理坐标系来表示高精地图(HD Map)中各个元素的地理位置。全球地理坐标系通常用途是用来表示纬度，经度和海拔。Apollo采用的是WGS84(World Geodetic System 1984)作为标准坐标系来表示物体的纬度和经度。通过使用该标准坐标系统，可以使用两个数字：x坐标，y坐标来唯一确定地球表面上除北极点之外的所有点，其中x坐标表示经度，y坐标表示纬度。全球地理坐标系的定义如图所示：
![](apollo-sensor_calibration\coordination_01.png)

### **局部坐标系-东-北-天(East-North-Up ENU)**:

在Apollo中，局部坐标系的定义为：

z轴-指向上方（与重力线为同一直线）
y轴-指向北面
x轴-指向东面

![](apollo-sensor_calibration\coordination_02.png)

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

<img src="apollo-sensor_calibration\coordination_04.png" style="zoom:67%;" />



### 传感器等硬件安装位置

## 2. 多传感器标定







### 多传感器数据融合 关系

## 3. Transform Wrapper





## 4. Apollo提供的标定工具

参考文件：docs/quickstart/multiple_lidar_gnss_calibration_guide_cn.md

