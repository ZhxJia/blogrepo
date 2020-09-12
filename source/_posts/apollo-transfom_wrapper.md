---
title: Apollo坐标转换组件
categories:
- autonomous
- apollo
tags:
- apollo
mathjax: true

---

Apollo 中的坐标转换组件 Transform和感知部分transform wrapper的相关内容

<!--more-->

Transform部分的文件依赖关系如下：
`third_party/tf2(ros) -> modules/transform -> perception/onboard/transform_wrapper

Tranform组件的原型为`ros/tf2`包，本文首先介绍modules/transrom组件的相关内容，然后针对感知部分介绍
transform_wrapper的相关功能。

## 1  Static Transform Component

转换关系存在静态(static)和动态转换，所谓静态转换，即转换关系是固定的，一般是物体之间是刚性连接，位置不会发生改变，这样的转换关系正常只需要发送一次即可；而对于动态转换关系，需要实时的发布自己的转换关系，涉及到时间戳的问题(比如由于车辆运动，则世界坐标系下车辆的位置一直在变化)。

组件结构：

```c++
class StaticTransformComponent final : public apollo::cyber::Component<> {
 public:
  StaticTransformComponent() = default;
  ~StaticTransformComponent() = default;

 public:
  bool Init() override;

 private:
  void SendTransforms(); //发送变换
  void SendTransform(const std::vector<TransformStamped>& msgtf);
  bool ParseFromYaml(const std::string& file_path, TransformStamped* transform); //从yaml中解析数据

  apollo::static_transform::Conf conf_; //pb.txt配置文件
  std::shared_ptr<cyber::Writer<TransformStampeds>> writer_; //消息发布句柄
  TransformStampeds transform_stampeds_; //外参数格式定义
};
```



### 1.1 配置文件

​	首先来看该组件的文件结构和配置文件的信息：

- `transform.proto`

  对应着内外参配置文件的参数组成，例如`modules/perception/data/params/front_6mm_extrinsics.yaml`

  ```protobuf
  message Transform {
      optional apollo.common.Point3D translation = 1;
      optional apollo.common.Quaternion rotation = 2;
  }
  
  message TransformStamped {
      optional apollo.common.Header header = 1;
      optional string child_frame_id = 2;
      optional Transform transform = 3;
  }
  
  message TransformStampeds {
      optional apollo.common.Header header = 1;
      repeated TransformStamped transforms = 2;
  }
  ```

- `static_transform_conf.proto`
  对应着所有静态转换的外参文件路径,例如：

  ```protobuf
  message ExtrinsicFile {
      optional string frame_id = 1; # destination sensor
      optional string child_frame_id = 2; # source sensor
      optional string file_path = 3;
      optional bool enable = 4;
  }
  
  message Conf {
      repeated ExtrinsicFile extrinsic_file = 1;
  }
  ```

  具体包含各传感器外参配置路径的文件为：`modules/transform/conf/static_transform_conf.pb.txt`

  **外参是在 destination sensor 下 source sensor的坐标，但是对应的是将source sensor下的检测信息转换到destination sensor下**

### 1.2 Static TF 初始化

**Init()**

初始化部分根据`dag`文件中的配置文件`static_transform_conf.pb.txt`进行参数的相应初始化，然后创建消息发布的句柄`node_->CreateWriter<TransformStampeds>(attr)` ,消息发布的格式为`transform.proto`中`TransformStampeds`,消息的通道名称由`adapter_gflags.cc`中的标志位确定：

```c++
DEFINE_string(tf_static_topic, "/tf_static", "Transform static topic.");
```

- **SendTransforms()**

  对`static_transform_conf.pb.txt`中所有的外参文件路径进行遍历，并解析对应的外参文件(.yaml)将对应的参数广播。

  - `PraseFromYaml(extrinsic_file.file_path,&transform)` ,将解析的参数对应到发布消息的格式数据类型中。

  - `SendTransform(transform_stamped_vec)` ,transfrom_stamped_vec为yaml解析得到的各外参文件的参数，将源传感器(child_frame_id)不同的外参添加到`transfrom_stampeds_`, 通过该重构的函数发布到`/tf_static`

    ```c++
    writer_->Write(std::make_shared<TransformStampeds>(transform_stampeds_));
    ```

    这里需要注意的是，child_frame_id一样的外参会被覆盖，child_frame_id不一样的外参会新建，所以要在写yaml外参文件时要注意不同传感器的child_fram_id不能重复，且child_frame_id以yaml中的为准。

    ```c++
      for (auto it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in) {
        bool match_found = false;
        for (auto& it_msg : *transform_stampeds_.mutable_transforms()) {
          if (it_in->child_frame_id() == it_msg.child_frame_id()) {
            it_msg = *it_in; //child_frame_id 一致则会覆盖
            match_found = true;
            break;
          }
        }
        if (!match_found) {
          *transform_stampeds_.add_transforms() = *it_in;
        }
      }
    ```

  <img src="apollo-transfom_wrapper\transform.jpg" style="zoom: 33%;" />
  
  ​						 图片来源于：https://github.com/daohu527/Dig-into-Apollo/tree/master/transform

## 2  transform_broadcaster

各个模块通过广播的方式发布动态变换，`transform broadcaster`提供了用于发布坐标转换信息的函数，其并没有作为一个单独的组件运行，而是编译为一个lib库供其他程序调用。

```protobuf
# `modules/transform/BUILD`
cc_library(
    name = "transform_broadcaster_lib",
    srcs = [
        "transform_broadcaster.cc",
    ],
    hdrs = [
        "transform_broadcaster.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/transform/proto:transform_proto",
    ],
)
```

该功能类包含了三个函数：
构造函数`TransformBroadcaster()`，消息发布函数`SendTransform(transform)`,`SendTransform(transforms)`

- `TransformBroadcaster(&node)`:构造函数需要传入`cyber::Node`节点，用于通过该传入的节点发布信息，发布通道为定义在`adapter_gflags.cc`中的`/tf`，发布消息的格式为`TransformStampeds`（定义于`transform.proto`中）

  ```c++
  writer_ = node_->CreateWriter<TransformStampeds>(attr);
  ```

- `SendTransform(transform)`:将`TransformStamped`格式的消息添加到向量`TransformStampeds`中，然后通过writer发布。

  ```c++
  void TransformBroadcaster::SendTransform(
    	const std::vector<TransformStamped>& transforms) {
    auto message = std::make_shared<TransformStampeds>();
    *message->mutable_transforms() = {transforms.begin(), transforms.end()};//将已有信息添加到message
    writer_->Write(message);
  }
  ```

## 3  Buffer

​	`buffer`提供了一个工具类给其他模块，主要作用是接收`/tf`和`/tf_static`通道的消息，并保持在buffer中，提供给其它节点进行查找和转换到对应的坐标系，buffer类继承了`BufferInterface`和`tf2::BufferCore`类。

### 3.1 BufferInterface 

​	定义了Buffer类的接口，拓展了TFCore类和TFCpp 类，主要接口函数有：

```c++
  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0
   *will get the latest)
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const cyber::Time& time, const float timeout_second = 0.01f) const = 0;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the
   * transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  virtual bool canTransform(const std::string& target_frame,
                            const std::string& source_frame,
                            const cyber::Time& time,
                            const float timeout_second = 0.01f,
                            std::string* errstr = nullptr) const = 0;

// Transform, advanced api, with pre-allocation
  template <typename T>
  T& transform(const T& in, T& out, const std::string& target_frame,  // NOLINT
               const cyber::Time& target_time, const std::string& fixed_frame,
               float timeout = 0.0f) const {
    // do the transform
    tf2::doTransform(
        in, out,
        lookupTransform(target_frame, target_time, tf2::getFrameId(in),
                        tf2::getTimestamp(in), fixed_frame, timeout));
    return out;
  }
```

以上仅列出每种功能的一个实现方法，主要就是实现了两种功能`lookupTransform`查找转换关系,`canTransform`是否可以转换

### 3.2 buffer

 除了上面的`BufferInterface `类，buffer还继承了ros/tf2中的`tf2:：BufferCore`。需要注意buffer为单例模式创建，接收转换消息，当其他模块用到转换时，从buffer中查询是否存在转换关系，并返回相应的转换。

- **Buffer::Init()** :`buffer`初始化，创建了节点`node_`，节点名称为
  `"transform_listener_" + std::to_string(cyber::Time::Now().ToNanosecond())`

  在该节点上首先创建了`Reader`用于监听`/tf`通道发布的动态坐标转换的消息。
  同时又创建了一个`Reader`用于监听`/tf_static`通道发布的静态坐标信息。
  回调函数均为`void Buffer::SubscriptionCallbackImpl(...,is_static)` is_static用于表示是否是静态坐标。

- **Buffer::SubscriptionCallbackImpl()**: 回调函数，接收处理`/tf` `/tf_static`通道的信息
  首先判断如果当前时刻小于上次更新的时刻，说明发生了跳变，此时清空TF buffer，然后重新加载静态转换关系。

  ```c++
    if (now.ToNanosecond() < last_update_.ToNanosecond()) {
      AINFO << "Detected jump back in time. Clearing TF buffer.";
      clear();
      // cache static transform stamped again.
      for (auto& msg : static_msgs_) {
        setTransform(msg, authority, true);
      }
    } //正常执行应该不会触发，如果是多线程则可能触发
  ```

  然后从监听到的`<Transformateds>`(transorm.proto)格式的消息中提取`header`,`child_frame_id`,`transform`等信息转存到`geometry_msgs::TransformStamped`(tf2格式)中（临时保存到trans_stamped），如果是静态坐标转换，则将信息添加到`static_msgs_`列表中保存,用于开始检查的时候重置时，重新加载静态转换关系时使用。

  - **BufferCore::setTransform(trans_stamped,authority,is_static)**
    调用tf2的函数，保存转换关系到`cache`中

    ```c++
    TimeCacheInterfacePtr frame = getFrame(frame_number);
        if (frame == NULL) frame = allocateFrame(frame_number, is_static);
    
        if (frame->insertData(TransformStorage(
                stripped, lookupOrInsertFrameNumber(stripped.header.frame_id),
                frame_number))) {
          frame_authority_[frame_number] = authority;
        }
    ```

    这部分基本都是调用的tf2包中的函数实现，而buffer相当于一个接口。

  - **Buffer::canTransform**
    在超时时间`timeout_second`时间范围内，每3000us执行下列函数查询是否有可用的Transform
    
    - **BufferCore::canTransform**
  - **Buffer::lookupTransform**
    - **BufferCore::lookupTransform**  返回tf2_trans_stamped
  - **Buffer::TF2MsgToCyber(tf2_trans_stamped,trans_stamped)** 将tf2格式的信息转换为proto格式的信息
  
<img src="apollo-transfom_wrapper\all.jpg" alt="图片来源于https://github.com/daohu527/Dig-into-Apollo/tree/master/transform" style="zoom:50%;" />
  
  动态转换主要是关于世界坐标系到本地坐标系的转换，实现于定位模块中，而各传感器之间的转换主要是静态转换，动态转换和静态转换最后同意由buffer管理，需要用到转换关系的模块通过查询接口获得对应的转换关系。

## 4 Transform Wrapper

从该函数的`BUILD`文件中可以知道该函数文件依赖于`//modules/transform:tf2_buffer_lib` ，将感知部分用到的坐标转化关系的获取进行了进一步封装(封装了Buffer*),实现的功能如下：

```c++
  // Attention: must initialize TransformWrapper first
  bool GetSensor2worldTrans(double timestamp,
                            Eigen::Affine3d* sensor2world_trans,
                            Eigen::Affine3d* novatel2world_trans = nullptr);

  bool GetExtrinsics(Eigen::Affine3d* trans);

  // Attention: can be called without initlization
  bool GetTrans(double timestamp, Eigen::Affine3d* trans,
                const std::string& frame_id, const std::string& child_frame_id);

  bool GetExtrinsicsBySensorId(const std::string& from_sensor_id,
                               const std::string& to_sensor_id,
                               Eigen::Affine3d* trans);

 protected:
  //内部函数 封装了buffer中的canTransform,lookupTransform
  bool QueryTrans(double timestamp, StampedTransform* trans,
                  const std::string& frame_id,
                  const std::string& child_frame_id);
 TransformCache transform_cache_; //双端队列用于缓存transform信息
```

Transofrom缓存的数据结构：`TransformCache`

```c++
  void AddTransform(const StampedTransform& transform);
  bool QueryTransform(double timestamp, StampedTransform* transform,
                      double max_duration = 0.0);

  inline void SetCacheDuration(double duration) { cache_duration_ = duration; }
```

- **Init()**
  初始化主要将`frame_id`,`child_frame_id`赋值到类内成员变量，然后设置transform_cache的缓存间隔`SetCacheDuration()`

- **GetSensor2worldTrans(tiemstamp,sensor2world_trans,novatel2world_trans)**
  该函数要求先进行`Init()`,获取传感器和定位模块到世界坐标系的转换。
  如果传感器到定位模块的外参为空，则首先先获取传感器到定位模块的转换关系（静态转换）

  - **QueryTrans()** 返回`trans_novatel2world` ，参数`timestamp`,`frame_id`,`child_frame_id`

    - `Buffer::canTransform()`
    - `Buffer::lookupTransform(frame_id,child_frame_id,query_time)`

    将`lookupTransform`得到的旋转平移矩阵转移到`Eigen`，并保存到类中定义的数据格式
    `StampedTransform`中

    ```c++
    struct StampedTransform {
      double timestamp = 0.0;  // in second
      Eigen::Translation3d translation;
      Eigen::Quaterniond rotation;
    };
    ```

  这样获得了sensor2novatel的转换关系后，只需获得novatel2world的转换关系，就可以得到
  sensor2world的转换关系，因此与上面相似通过`QueryTrans`查询是否有对应当前时间戳的转换关系，查询成功获得`trans_novatel2world`则调用：

  - `transform_cache_.AddTransform(trans_novatel2world)`添加到缓存中。

  查询不成功的调用缓存中的数据`TransformCache::QueryTransform()`进行推断，但是要求推断的时间延迟间隔最多不超过0.15s，通过历史信息进行插值得到`trans_novatel2world`

  然后很容易就可以得到sensor2world的坐标变换矩阵。

  ```c++
  *sensor2world_trans = novatel2world * (*sensor2novatel_extrinsics_);
  ```

- **GetExtrinsics()**

  该函数使用需要先进行`Init()`在`GetSensor2worldTrans()`中获得，此处返回sensor2novatel的变换矩阵

- **GetTrans()**
  该函数执行不需要先执行`Init()`，获取任意的`frame_id`和`child_frame_id`的转换关系。

- **GetExtrinsicsBySensorId()**

  该函数执行不需要先执行`Init()`,首先根据传感器SensorId，通过SensorManager类查询得到对应的`frame_id`,然后通过`QueryTrans()`获得外参。

  

## 参考资料：

http://wiki.ros.org/tf2

https://github.com/daohu527/Dig-into-Apollo/tree/master/transform