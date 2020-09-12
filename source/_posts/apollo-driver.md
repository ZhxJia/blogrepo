---

title: apollo相机雷达的驱动
categories:
- autonomous
- apollo
tags:
- apollo
mathjax: true
---

apollo中Camera，Radar，Lidar，Can等驱动程序

<!--more-->

## 1 Camera

Camera包基于V4L USB相机设备实现封装，提供图像采集及发布的功能。

### 1.1 camera_component

**组件配置**
`modules/drivers/camera/dag/camera.dag`
每个相机建立一个组件，这里以`front_6mm`为例,对应于dag文件

```protobuf
module_config{
	module_library : "/apollo/bazel-bin/modules/drivers/camera/libcamera_component.so"
	
	components {
		class_name : "CameraComponent"
		config {
			name : "camera_front_6mm"
             config_file_path : "/apollo/modules/drivers/camera/conf/camera_front_6mm.pb.txt"
		}
	}
	...
}
```

由组件的配置文件路径可以得到该相机`front_6mm`组件对应于同级目录`proto/config.proto`中默认参数的修改。
**组件初始化：**
加载dag文件中config_file_path的参数`camera_config_`。
创建相机设备实例,并利用`camera_config_`来初始化该相机实例,相机类为`UsbCam`

```c++
apollo::cyber::common::GetProtoFromFile(config_file_path_,camera_config_.get());
camera_device_.reset(new UsbCam());
camera_device_.init(camera_config_);
raw_image.reset(new CameraImage);
```

这里用到了两个类`UsbCam`和`CameraImage` 。其中`UsbCam`表示相机设备，该类包含了相机的硬件配置参数(像素编码格式，帧率，宽高，曝光及其他一些配置)。`CameraImage`则表示相机的图像结构信息（包括了图像的宽高和存储信息)。

```c++
struct CameraImage {
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  int is_new;
  int tv_sec;
  int tv_usec;
  char* image;
  ~CameraImage() {
    if (image != nullptr) {
      free(reinterpret_cast<void*>(image)); //！！！！
      image = nullptr;
    }
  }  
  };
```

然后为CameraImage中的image分配内存（长度为imagesize）:

```c++
  raw_image_->image =
      reinterpret_cast<char*>(calloc(raw_image_->image_size, sizeof(char))); 
  //注意在CameraImage的析构函数中，一定不能忘记释放内存
```

根据`modules/driver/proto/sensor_iamge.proto`中定义的图像消息格式`<Image>`实例化`pb_iamge`进行相关参数的配置：

```c++
  for (int i = 0; i < buffer_size_; ++i) { //16
    auto pb_image = std::make_shared<Image>();
    pb_image->mutable_header()->set_frame_id(camera_config_->frame_id()); //camera_front_6mm
    pb_image->set_width(raw_image_->width);
    pb_image->set_height(raw_image_->height);
    pb_image->mutable_data()->reserve(raw_image_->image_size); // 分配空间给data

    if (camera_config_->output_type() == YUYV) {
      pb_image->set_encoding("yuyv");
      pb_image->set_step(2 * raw_image_->width); //每一行像素所占的空间
    } else if (camera_config_->output_type() == RGB) {
      pb_image->set_encoding("rgb8");
      pb_image->set_step(3 * raw_image_->width);
    }

    pb_image_buffer_.push_back(pb_image); //缓存16个
  } 
```

最后在该节点上创建writer用于图像消息的发布,channel:"/apollo/sensor/camera/front_6mm/image"

```c++
writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());
async_result_ = cyber::Async(&CameraComponent::run, this);
```

注意对于`CameraComponent`的运行使用了异步编程，执行的函数为`CameraComponent::run()`,这样各个摄像头组件将会在多线程中运行。

**组件运行(run)**
`CameraComponent::run()`函数用于发布图像信息，该函数通过异步的方式运行(while{}循环一直执行,间隔1/200s)

异步循环执行，首先查询相机设备的状态(是否正常连接)，如果未正常连接，等待2000ms再次检测。

```c++
    if (!camera_device_->wait_for_device()) {
      // sleep for next check
      cyber::SleepFor(std::chrono::milliseconds(device_wait_)); //device_wait_ms: 2000
      continue;
    }
```

然后通过`camera_device_->poll(raw_image_)`获取图像信息。

```c++
    if (!camera_device_->poll(raw_image_)) {
      AERROR << "camera device poll failed"; //轮询失败
      continue;
    }
```

最后将图像和时间信息添加到`<Image>`数据结构中，通过writer发布，等待5000us->5ms再次执行。每秒200帧。

```c++
message Image {
  optional apollo.common.Header header = 1;
  optional string frame_id = 2; //传感器名称id
  optional double measurement_time = 3;

  optional uint32 height = 4;  // image height, that is, number of rows
  optional uint32 width = 5;   // image width, that is, number of columns

  optional string encoding = 6;
  optional uint32 step = 7;  // Full row length in bytes   = bytes_per_pixel*width
  optional bytes data = 8;   // actual matrix data, size is (step * rows)
}
```

----

### 1.2 compress_component

`modules/drivers/camera/dag/camera.dag`
`compress_component`的dag配置文件,同样以`front_6mm`为例：

```protobuf
    components {
      class_name : "CompressComponent"
      config {
        name : "camera_front_6mm_compress"
        config_file_path : "/apollo/modules/drivers/camera/conf/camera_front_6mm.pb.txt"
        readers {
          channel: "/apollo/sensor/camera/front_6mm/image"
          pending_queue_size: 10
        }
      }
    }
    ...
```

该组件接收camera_component发布的图像信息然后进行压缩，等待队列的大小设置为10。
**组件初始化：**
加载图像压缩的配置信息，以camera_front_6mm.pb.txt中为例：

```c++
compress_conf {
    output_channel: "/apollo/sensor/camera/front_6mm/image/compressed"
    image_pool_size: 100 //对象池的大小
}
```

初始化`std::shared_ptr<CCObjectPool<CompressedImage>> image_pool_;`对象，这里用到对象池，在需要某个类的多个实例时使用，此处对象池的大小为100。
创建节点writer,通道名称为`output_channel`:

```c++
 writer_ = node_->CreateWriter<CompressedImage>(
      config_.compress_conf().output_channel());
```

**组件处理程序Proc**
当接收到原始的图像信息，设置`frame_id`,`header`,`measurement_time`与原图像信息一致，不同的是compress特有的
`format`设置为：`image->encoding() + "; jpeg compressed bgr8"`  

> 原始图像的encoding为配置文件中的 output_type决定，默认为RGB

接下来就是通过opencv将图像重新编码压缩

```c++
  std::vector<int> params;
  params.resize(3, 0);
  params[0] = CV_IMWRITE_JPEG_QUALITY;
  params[1] = 95; //压缩图像 100表示不压缩
```

利用图像编码函数,压缩为jpeg格式，且RGB->bgr8

```c++
cv::imencode(".jpg", tmp_mat, compress_buffer, params);
compressed_image->set_data(compress_buffer.data(), compress_buffer.size());
```

然后将压缩的图像通过writer发布,通道名称为`"/apollo/sensor/camera/front_6mm/image/compressed"`
但是貌似好像实际`FusionCameraDetectionComponent`的接收图像信息还是未压缩的图像，即通道：
`"/apollo/sensor/camera/front_6mm/image"` ,但是高清地图信息采集时貌似用的时压缩的图像。

```protobuf
message CompressedImage {
  optional apollo.common.Header header = 1;
  optional string frame_id = 2; //对应的传感器的名称

  // Specifies the format of the data
  //  Acceptable values: jpeg, png
  optional string format = 3; //压缩的图像类型,eg. jpeg
  optional bytes data = 4;  // Compressed image buffer
  optional double measurement_time = 5;
  optional uint32 frame_type = 6;
}
```



----

### 1.3 video_component

首先看一下该组件的dag配置文件：(基本被注释了，表示正常情况下，该组件并没有运行)

```c++
# Define all coms in DAG streaming.
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/video/libvideo_driver_component.so"

#    components {
#      class_name : "CompCameraH265Compressed"
#      config {
#        name : "camera_front_6mm_compress"
#        config_file_path : "/apollo/modules/drivers/video/conf/video_front_6mm.pb.txt"
#      }
#    }

#    components {
#      class_name : "CompCameraH265Compressed"
#      config {
#        name : "camera_front_12mm_compress"
#        config_file_path : "/apollo/modules/drivers/video/conf/video_front_12mm.pb.txt"
#       }
#    }

}
```

**组件初始化**
根据配置文件创建`CameraDriver`实例`camera_device_`，然后进行CameraDriver初始化：

- `CameraDriver::Init()`
  创建`ScoketInput`实例，并通过配置文件中的`udp_port`进行初始化，初始化`sock_fd`和`port`,即生成套接字文件描述符和端口号

> udp_port：为UDP 数据传输协议的端口，程序中从2000开始，每一个相机递增1

- `CameraDriver::Record()`返回配置文件中的record，若值为1，则使用环境变量`H265_SAVE_FOLDER`（当环境变量没有设置，则使用当前路径位置）位置创建文件路径用于保存record文件。
- 创建`<CompressedImage>`实例，并创建节点`writer`用于传输压缩的图像数据。
- 创建线程执行`CompCameraH265Compressed::VideoPoll`

**组件处理函数**
循环执行下列程序，直到系统退出。

- `CameraDriver::Poll(std::shared_ptr<CompressedImage> h265)`

  - `PollByFrame(h265)`  轮询获取压缩图像信息

    - `SocketInput::GetFramePacket(h265)` 通过socket底层库`recvfrom`经socket获取数据到缓冲区`pdu_data`

      ```c++
      //成功则返回实际接收到的字符数，失败返回-1
      ssize_t pdu_len = recvfrom(sockfd_, pdu_data, H265_PDU_SIZE, 0, NULL, NULL);
      ```

然后将接收到压缩图像通过节点发布，并记录二进制图像数据到record文件中，文件名：`encode_{端口号}.h265`

-----

### 1.4 image_decompress

组件的`dag`配置文件：

```protobuf
module_config {
    module_library : "/apollo/bazel-in/modules/drivers/tools/image_decompress/libimage_decompress.so"
    components {
      class_name : "ImageDecompressComponent"
      config {
        name : "camera_front_6mm_decompress"
        config_file_path : 	
        	   "/apollo/modules/drivers/tools/image_decompress/conf/camera_front_6mm.pb.txt"
        readers {
          channel: "/apollo/sensor/camera/front_6mm/image/compressed"
          pending_queue_size: 10
        }
      }
    ...
```

该组件接收压缩图像通道的信息，并进行解压缩，最后将图像信息发布到`"/apollo/sensor/camera/front_6mm/image"`通道中。

**组件初始化**
创建该节点的消息发布通道例如，`channel_name: "/apollo/sensor/camera/front_6mm/image"`

**组件处理函数Proc**
当接受到`"/apollo/sensor/camera/front_6mm/image/compressed"`通道发布的压缩图像数据，进行解压缩操作：
方法是压缩图像(Compressed Component)的逆向操作，通过opencv函数`cv::imdecode()`解码获得cv::Mat格式图像,然后将图像由bgr转换为rgb，最后通过writer发布出去。

编码解码的用到的函数有：

1. 将原始的`<Image>`格式的图像转存到`cv::Mat`数据结构中。

   ```c++
   cv::Mat mat_image(image->height(), image->width(), CV_8UC3,
                         const_cast<char*>(image->data().data()), image->step());
   //注意image data在proto中的定义为bytes,实际c++中对应std::string(可以包含任意顺序的字节数据)，
   //string.data()返回char*指向数据的首地址，这里Mat的构造方法
   //Mat::Mat(int rows, int cols, int type, void* data, size_t step=AUTO_STEP) 
   //step指一行元素的字节数
   ```

2. 将Mat格式图像编码为`std::vector<uint8_t>`压缩格式(byte格式的数组，用于方便传输)，即将图像数据转为二进制数据。

   ```c++
   std::vector<uint8_t> compress_buffer;
   cv::imencode(".jpg", tmp_mat, compress_buffer, params)
   ```

   

3. 将byte格式的数组解码恢复到Mat,将内存中的二进制图像数据转换为opencv可处理的格式，首先创建一个byte数组用于存储二进制数据，

   ```c++
     std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                              compressed_image->data().end());
     cv::Mat mat_image = cv::imdecode(compressed_raw_data, CV_LOAD_IMAGE_COLOR);
   ```

4. 最后将Mat结构数据转存到内部数据结构中`<Image>`中

   ```c++
     auto size = mat_image.step * mat_image.rows;
     image->set_data(&(mat_image.data[0]), size);
   ```



**注意：** **std::string 与字符数组并不是意味着文本格式，而是将string 作为二进制存储的容器。**

参考：https://blog.csdn.net/qq_37406130/article/details/78820176?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task

-------------

### 1.5 启动脚本

```bash
# in docker
bash /apollo/scripts/camera.sh
# or
cd /apollo && cyber_launch start modules/drivers/camera/launch/camera.launch
```

如果报错： `sh: 1: v4l2-ctl: not found`,需要安装v4l2库。

```bash
sudo apt-get install v4l-utils
```

如果图像被压缩了，运行image decompression module:

```bash
cyber_launch start modules/drivers/tools/image_decompress/launch/image_decompress.launch
```

----

### 参考

https://www.cnblogs.com/hgl0417/p/9190835.html

v4l2 API：
https://baike.baidu.com/item/V4L2?sefr=enterbtn

https://www.cnblogs.com/aquafly/p/6474555.html

颜色空间的表示方法
https://www.cnblogs.com/x_wukong/p/4919774.html

c++异步编程：
https://blog.csdn.net/u012372584/article/details/97108417

并发对象池：
http://ifeve.com/generic-concurrent-object-pool/

Opencv压缩图像格式：
https://www.jianshu.com/p/fed6a8a99625

UDP socket

https://www.cnblogs.com/HpeMephisto/p/11312193.html

protobuf :序列化与反序列化

https://cloud.tencent.com/developer/article/1176660