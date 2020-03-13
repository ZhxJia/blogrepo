---

title: Apollo中radar的相关算法
categories:
- apollo
- perception
tags:
- radar
- perception
mathjax: true
---

apollo中radar的目标检测与跟踪算法

<!--more-->

Apollo中`radar_detection_component`的算法处理入口为`Proc` ,根据该组件对应`dag`文件中的`reader`,在初始化组件时，`class RadarDetectionComponent : public cyber::Component<ContiRadar>`，该组件继承了含有一个message消息的`Component`基类，因此在该类初始化时，会创建接收`ContiRadar`消息类型的`reader`，回调函数为`Proc`。

Proc的输入信息(input_message)包括了：

- `apollo::drivers::ContiRadar`
- `apollo::localization::LocalizationEstimate`

输出信息(out_message)包括了：

- `SensorFrameMessage` 



## 预处理

```c++
  // @brief: correct radar raw obstacles.
  // @param [in]: raw obstacles from radar driver.
  // @param [in]: options.
  // @param [out]: corrected radar obstacles
  virtual bool Preprocess(const drivers::ContiRadar& raw_obstacles,
                          const PreprocessorOptions& options,
                          drivers::ContiRadar* corrected_obstacles) = 0;
```



- SkipObjects(raw_obstacles,corrected_obstacles)

  ```c++
  // @brief: 根据时间戳过滤检测物体
  // @param [in]: raw obstacles from radar driver.
  // @param [out]: corrected radar obstacles
  void ContiArsPreprocessor::SkipObjects(
      const drivers::ContiRadar& raw_obstacles,
      drivers::ContiRadar* corrected_obstacles){...}
  ```

  判断每个原检测物体的时间戳，若该检测物体的时间戳在该数据帧时间戳的`(-1e6,0.074)`之间则进行处理。

-  ExpandIds(corrected_obstacles)

  ```c++
  // @brief: 创建全局id
  // @param [in]: corrected radar obstacles
  // @param [out]: corrected radar obstacles
  void ContiArsPreprocessor::ExpandIds(drivers::ContiRadar* corrected_obstacles) {...}
  ```

  为corrected obstacles分配id:若目标的`meas_state`属性为`CONTI_NEW`即新创建的目标或者corrected obstacles的id未出现过，则为其添加一个，最终当所有的object_id都出现过`local2global_[ORIGIN_CONTI_MAX_ID_NUM]`将不再变化，同时每个object_id都对应一个固定的id。

- CorrectTime(corrected_obstacles)

  ```c++
  // @brief: 校正时间戳
  // @param [in]: corrected radar obstacles
  // @param [out]: corrected radar obstacles
  void ContiArsPreprocessor::CorrectTime(
      drivers::ContiRadar* corrected_obstacles) {...}
  ```

  通过初始化配置时的参数`delay_time_`(0.07)校正时间戳。

