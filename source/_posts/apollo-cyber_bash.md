---

title: Apollo中Cyber RT脚本启动流程
categories:
- apollo
- Cyber RT
tags:
- apollo
mathjax: true
---

Apollo中Cyber RT组件脚本启动流程

<!--more-->



## 1. CyberRT功能模块启动过程

Apollo通过组件注册的形式构建

https://zhuanlan.zhihu.com/p/116782645





## 2. Cyber API

### 2.1 Talk-Listener

首先引入三个概念：`node`(base unit),`reader`(facility to read message),`writer` (facility to write message)，即`reader`和`writer`是建立在`node`上的。
**创建node**：
在CyberRT框架下，node是最基本的单元（wirter和reader的基础），创建node的接口如下:

```c++
std::unique_ptr<Node> apollo::cyber::CreateNode(const std::string& node_name, const std::string& name_space = "");
```

- Parameters:
  - node_name:`node`的名称，需要是全局唯一的标识符（名字不能重复）
  - name_space: 表明`node`位于的命名空间,默认为空，它最终与`node_name`相连接作为最终节点名，形式
    是`/namespace/node_name`
- Return value - 指向`Node`的指针(unique_ptr)
- Error Conditions - 当`Cyber::Init()`没有执行，系统没有进行初始化，则不能创建节点，会返回`nullptr`

> 注意，一般一个功能组件仅仅只有一个node,但是在node上可以创建多个writer和reader。

**创建writer**:
`writer`是CyberRT中用于消息发送的"设备"，每一个`writer`都与一个对应特定数据类型的`channel`关联。`writer`通过`Node`类中的`CreateWriter`接口创建，接口(位于node.h)如下：

```c++
template <typename MessageT>
	auto CreateWriter(const std::string& channel_name)
		-> std::shared_ptr<Writer<MessageT>>;
template <typename MessageT>
	auto CreateWriter(const proto::RoleAttributes& role_attr)
		-> std::shared_ptr<Writer<MessageT>>;
```

- Parameters:
  - channel_name: 发布消息的channel名称
  - MessageT:发送消息的类型
- Return value - 指向`Writer`类实例的指针

**创建reader:**
`reader`的创建与`writer`类似，是CyberRT中用于接收消息的"设备"，`reader`在创建的时候必须与一个回调函数绑定，当channel中新的消息到来时，将会调用回调函数。`reader`的创建的函数接口为(node.h)：

```c++
template <typename MessageT>
auto CreateReader(const std::string& channel_name,const std::function<void(const std::shared_ptr<MessageT>&)>& reader_func)
	-> std::shared_ptr<Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const ReaderConfig& config,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
    -> std::shared_ptr<cyber::Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const proto::RoleAttributes& role_attr,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
-> std::shared_ptr<cyber::Reader<MessageT>>;
```

> CallbackFunc 封装了`std:function<void(const std::shared_ptr<M0>&)>`

- Parameters:
  - MessageT: 读取的消息类型
  - channel_name:对应接收消息来自哪个channel
  - reader_func:回调函数用于处理接收到的消息
- Return value:指向Reader类实例的指针

----

### 2.2 Service Creation and Use

在自动驾驶场景中，往往需要多个模块之间的信息交换，除了上述的接收和发送消息的方式。`Service`是`node`间通信的另一种方式。与channel的方式不同，`service`实现了`two-way`交流，例如一个node通过发送一个请求来获得一个回复。

下面举例说明通过创建`client-server`模型将`Driver.proto`信息来回传递，当client发送一个请求，对应的server解析或处理请求并进行回复。通过以下几个步骤即可实现：

**定义request and response messages**
在cyber中所有的信息传递通过`protobuf`的形式传递。任何的protobuf消息通过序列化/反序列化(相关函数接口实现)可以作为`service`的请求 回复的消息格式，例如这里采用的Driver消息类型位于examples.proto:

```c++
// filename: examples.proto
syntax = "proto2";
package apollo.cyber.examples.proto;
message Driver {
    optional string content = 1;
    optional uint64 msg_id = 2;
    optional uint64 timestamp = 3;
};
```

**Create a service and a client**

```c++
//filename: cyber/examples/service.cc
#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;

int main(int argc ,char* argv[]){
    apollo::cyber::Init(argv[0]);
    std::shared_ptr<apollo::cyber::Node> node(apollo::cyber::CreatNode("strat_node"));
    atuo server = node->CreateService<Driver,Driver>("test_server",[](const std::shared_ptr<Driver>& request,std::shared_ptr<Drive>& response)){
        AINFO << "server: I am driver server";
        static uint64_t id = 0;
        ++id;
        response->set_msg_id(id);
        response->set_timestamp(0);
      }); //lambda函数实现回调信息
    
      auto client = node->CreateClient<Driver, Driver>("test_server");
  auto driver_msg = std::make_shared<Driver>();
  driver_msg->set_msg_id(0);
  driver_msg->set_timestamp(0);
  while (apollo::cyber::OK()) {
    auto res = client->SendRequest(driver_msg);
    if (res != nullptr) {
      AINFO << "client: response: " << res->ShortDebugString();
    } else {
      AINFO << "client: service may not ready.";
    }
    sleep(1);
  }

  apollo::cyber::WaitForShutdown();
  return 0
}
```

**Bazel编译文件的构建：**

```c++
cc_binary(
    name = "service",
    srcs = [ "service.cc", ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)
```

**编译运行：**

- 编译service/client: `bazel build cyber/examples/...`
- 运行：`./bazel-bin/cyber/examoles/service`
- 结果：`apollo/data/log/service.INFO`中可以查看`AINFO`的信息。

**注意事项：**

- 在注册`service`，`node`时 注意不能出现重复的名称

----

### 2.3 Parameter service

参数服务用于节点之间的参数共享，提供的基本操作为`set`,`get`以及`list`。参数共享是基于上面`Service`(包含`service`和`client`)实现的。

**Parameter Object:**
**（1）支持的数据类型：**
通过cyber的所有参数传递是基于`apollo::cyber::Parameter`类型，下表给出了支持的5种参数类型：

| 参数类型                                  | C++数据类型 | protobuf数据类型 |
| ----------------------------------------- | ----------- | ---------------- |
| apollo::cyber::proto::ParamType::INT      | int64_t     | int64            |
| apollo::cyber::proto::ParamType::DOUBLE   | double      | double           |
| apollo::cyber::proto::ParamType::BOOL     | bool        | bool             |
| apollo::cyber::proto::ParamType::STRING   | std::string | string           |
| apollo::cyber::proto::ParamType::PROTOBUF | std::string | string           |
| apollo::cyber::proto::ParamType::NOT_SET  | -           | -                |

除了以上五种类型，Parameter同时还有支持protobuf对象作为输入参数的接口。通过序列化处理对象将其转换为STRING类型用于传递。
**（2）创建Parameter对象：** 
支持的构造方法

```c++
Parameter();  // Name is empty, type is NOT_SET
explicit Parameter(const Parameter& parameter);
explicit Parameter(const std::string& name);  // type为NOT_SET
Parameter(const std::string& name, const bool bool_value);
Parameter(const std::string& name, const int int_value);
Parameter(const std::string& name, const int64_t int_value);
Parameter(const std::string& name, const float double_value);
Parameter(const std::string& name, const double double_value);
Parameter(const std::string& name, const std::string& string_value);
Parameter(const std::string& name, const char* string_value);
Parameter(const std::string& name, const std::string& msg_str,
          const std::string& full_name, const std::string& proto_desc);
Parameter(const std::string& name, const google::protobuf::Message& msg);
```

具体使用举例：

```c++
Parameter a("int", 10);
// proto message Chatter
Chatter chatter;
Parameter f("chatter", chatter);
```

**（3）接口和数据读取：**

```c++
inline ParamType type() const;
inline std::string TypeName() const;
inline std::string Descriptor() const;
inline const std::string Name() const;
inline bool AsBool() const;
inline int64_t AsInt64() const;
inline double AsDouble() const;
inline const std::string AsString() const;
std::string DebugString() const;
```

```c++
Parameter a("int", 10);
a.Name();  // return int
a.Type();  // return apollo::cyber::proto::ParamType::INT
a.TypeName();  // return string: INT
a.DebugString();  // return string: {name: "int", type: "INT", value: 10}
int x = a.AsInt64();  // x = 10
x = a.value<int64_t>();  // x = 10
x = a.AsString();  // Undefined behavior, error log prompt
f.TypeName();  // return string: chatter
auto chatter = f.value<Chatter>();
```

**Parameter Service**
如果一个node 想要提供Parameter Service给其他节点，则需要创建`ParameterService`

```c++
/**
 * @brief Construct a new ParameterService object
 *
 * @param node shared_ptr of the node handler
 */
explicit ParameterService(const std::shared_ptr<Node>& node);
```

因为参数存储在`parameter service object`中，因此可以在ParameterService直接处理，而不需要service request。
**(1）设置parameter对象：**

```c++
/**
 * @brief Set the Parameter object
 *
 * @param parameter parameter to be set
 */
void SetParameter(const Parameter& parameter);
```

**(2) 获取parameters对象中参数：**

```c++
/**
 * @brief Get the Parameter object
 *
 * @param param_name
 * @param parameter the pointer to store
 * @return true
 * @return false call service fail or timeout
 */
bool GetParameter(const std::string& param_name, Parameter* parameter);
```

**（3）获取parameters对象列表：**

```c++
/**
 * @brief Get all the Parameter objects
 *
 * @param parameters pointer of vector to store all the parameters
 * @return true
 * @return false call service fail or timeout
 */
bool ListParameters(std::vector<Parameter>* parameters);
```

**Parameter Client:**
如果一个node想要使用其他节点的parameter services,则需要创建一个`ParamterClient`

```c++
/**
 * @brief Construct a new ParameterClient object
 *
 * @param node shared_ptr of the node handler
 * @param service_node_name node name which provide a param services
 */
ParameterClient(const std::shared_ptr<Node>& node, const std::string& service_node_name);
```

与`Parameter Service`类似，也可以使用`SetParameter`,`GetParameter`和`ListParamters`。

```c++
int main(int argc, char** argv) {
  apollo::cyber::Init(*argv);
  std::shared_ptr<apollo::cyber::Node> node =
      apollo::cyber::CreateNode("parameter");
  auto param_server = std::make_shared<ParameterServer>(node);
  auto param_client = std::make_shared<ParameterClient>(node, "parameter");
  param_server->SetParameter(Parameter("int", 1));
  Parameter parameter;
  param_server->GetParameter("int", &parameter);
  AINFO << "int: " << parameter.AsInt64();
  param_client->SetParameter(Parameter("string", "test"));
  param_client->GetParameter("string", &parameter);
  AINFO << "string: " << parameter.AsString();
  param_client->GetParameter("int", &parameter);
  AINFO << "int: " << parameter.AsInt64();
  return 0;
}
```

-----

### 2.4 Log API

Cyber的log库建立在`glog`的基础上，需要包含以下头文件：

```c++
#include "cyber/common/log.h"
#include "cyber/init.h"
```

**Log configuration:**
默认的配置路径`cyber/setup.sh`
以下的配置信息可以由开发者自行更改：

```bash
export GLOG_log_dir=/apollo/data/log
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
```

**Log initialization:**
在代码入口调用`Init`来初始化log:

```c++
apollo::cyber::cyber::Init(argv[0]) is initialized.
If no macro definition is made in the previous component, the corresponding log is printed to the binary log.
```

**Log output macro:**
封装的log输出宏：(严重程度依次递推)

```c++
ADEBUG << "hello cyber.";
AINFO  << "hello cyber.";
AWARN  << "hello cyber.";
AERROR << "hello cyber.";
AFATAL << "hello cyber.";
```

**Log输出Format:**
`<MODULE_NAME>.log.<LOG_LEVEL>.<datetime>.<process_id>`

**关于Log文件：**
与原始`glog`不一样的仅仅在于不同的log级别将会写入相同的log文件中。

-----

### 2.5 Building a module based on Component

#### 2.5.1 component

`component`是CyberRT提供用于构建应用模块的基础类。每一个特定功能的应用模块都可以继承`Component`类并定义它们自己的`Init`和`Proc`函数，这样该模块将会被加载到`Cyber`框架中运行。

#### 2.5.2 Binary vs Component

在CyberRT框架下应用有两种配置方式：

- 基于Binary:各个应用模块被分别编译成不同的binary(二进制可执行文件)，然后通过创建各自的`Reader`和`Writer`进行信息交换。

- 基于Component:各个应用模块被编译成shared library。通过继承Component类,并编写相应的dag描述文件,CyberRT框架将会动态的加载运行该应用模块。

  **Component的基本组件接口：**

  - `Init()` 用于算法组件的初始化，是算法组件的入口(相当于各个组件的main)
  - `Proc()` 当需要的通道信息发布时，Cyber框架将会调用该函数(实际是一个Reader的回调函数)

  **使用Component的优势：**

  - 组件可以通过`launch`文件加载到不同的进程中，部署更加灵活
  - 组件可以通过修改`dag`文件更改接收通道的名称，而不需要重新编译
  - 组件支持接收多种类型的数据
  - 组件支持提供多种融合策略

#### 2.5.3 Dag文件的形式

An example dag file:

```
# Define all coms in DAG streaming.
module_config {
    module_library : "lib/libperception_component.so"
    components {
        class_name : "PerceptionComponent"
        config {
            name : "perception"
            readers {
                channel: "perception/channel_name"
            }
        }
    }
    timer_components {
        class_name : "DriverComponent"
        config {
            name : "driver"
            interval : 100
        }
    }
}
```

- **module_library:** 用于加载`.so`库的路径，其根路径是Cyber的工作路径(与`setup.sh`相同的路径)
- **components/timer_components:** 选择基础组件类型(是否是定时发布,一般传感器的组件用timer_component)
- **class_name:** 加载的组件类的名称
- **name:** 加载的组件类的标识符（同一组件类可能有不同的配置方式，比如16线，64线激光雷达）
- **readers:** 被当前组件接受的数据(Proc函数中处理)，支持1-3各不同通道的数据

#### 2.5.4 (timer)component example

- common_component_example
  可以查看`cyber/examples/common_component_example/`中的示例程序
  头文件基本组成：

  ```c++
  #include <memory>
  #include "cyber/class_loader/class_loader.h"
  #include "cyber/examples/proto/examples.pb.h"
  
  using apollo::cyber::examples::proto::Driver;
  using apollo::cyber::Component;
  using apollo;:cyber::ComponentBase;
  
  class CommontestComponent : public Component<Driver,Driver> {
      public:
      	bool Init() override;
      	bool Proc(const std::shared_ptr<Driver>& msg0,
                    const std::shared_ptr<Driver>& msg1) override;
  };
  CYBER_REGISTER_COMPONENT(Commontestcomponent)  
  ```

  源文件：

  ```c++
  #include "cyber/examples/common_component_sample/common_component_example.h"
  
  #include "cyber/class_loader/class_loader.h"
  #include "cyber/component/component.h"
  
  bool Commontestcomponent::Init(){
      AINFO << "Commontest component init";
      return true;
  }
  bool Commontestcomponent::Proc(const std::shared_ptr<Driver>& msg0,
                                const std::shared_ptr<Driver>& msg1){
      AINFO << "Start commontest component Proc [" << msg0->msg_id() << "] ["
          << msg1->msg_id() << "]";
      return true;
  }
  ```

  

- timer_component_example
  参考`cyber/examples/timer_component_example/`
  头文件：

  ```c++
  #include <memory>
  
  #include "cyber/class_loader/class_loader.h"
  #include "cyber/component/component.h"
  #include "cyber/component/timer_component.h"
  #include "cyber/examples/proto/examples.pb.h"
  
  using apollo::cyber::examples::proto::Driver;
  using apollo::cyber::Component;
  using apollo::cyber::ComponentBase;
  using apollo::cyber::TimerComponent;
  using apollo::cyber::Writer;
  
  class TimertestComponent : public TimerComponent{
  public:
  	bool Init() override;
  	bool Proc() override;
      
  private:
  	std::shared_ptr<Writer<Driver>> driver_writer_ = nullptr;
  };
  CYBER_REGISTER_COMPONENT(TimertestComponent)
  
  ```

  源文件：

  ```c++
  #include "cyber/examples/timer_component_example/timer_component_example.h"
  
  #include "cyber/class_loader/class_loader.h"
  #include "cyber/component/component.h"
  #include "cyber/examples/proto/examples.pb.h"
  
  bool TimertestComponent::Init(){
  	driver_writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
  	return true;
  }
  
  bool TimertestComponent::Proc(){
      static int i = 0;
      auto out_msg = std::make_shared<Driver>();
      out_msg->set_msg_id(i++);
      driver_writer_->Write(out_msg);
      AINFO << "timertestcomponent: Write drivemsg->"
          <<out_msg->ShortDebugString();
      return true;
  }
  ```

  然后通过bazel编译BUILD文件生成`.so`文件，通过`timer.dag`即可运行组件

需要注意的是：

- 需要注册组件，才能够从通过`SharedLiabray`加载功能类，注册的接口：

  ```c++
  CYBER_REGISTER_COMPONENT(DriverComponent)
  ```

  在组件类的头文件的最后添加,如果在注册时使用了`namespace`，那么在dag文件中也要添加对应的`namespace`

- Component和TimerComponent的配置是不同的，不要将它们搞混

### 2.6 Launch

`cyber_launch`是CyberRT框架的启动程序。它根据`launch`文件启动`mainboards`进程(一个launch文件对应一个进程),并将不同的组件根据dag文件加载到不同的`mainboard`进程中。`cyber_launch`支持动态加载组件或启动二进制文件两种场景。
launch文件的格式：

```xml
<cyber>
	<module>
    	<name>driver</name>
        <dag_conf>driver.dag</dag_conf>
        <process_name></process_name>
        <exception_handler>exit</exception_handler>
    </module>
    <module>
    	<name>perception</name>
        <dag_conf>perception.dag</dag_conf>
        <process_name></process_name>
        <exception_handler>respawn</exception_handler>
    </module>
    <module>
    	<name>planning</name>
        <dag_conf>planning.dag</dag_conf>
        <process_name></process_name>
    </module>
</cyber>
```

**Module:** 每一个被加载的组件或二进制文件都是一个module

- **name** 被加载的模块的名称
- **dag_conf** 对应组件的dag文件
- **process_name** 是mainboard开启后对应的进程名称。相同的process_name的组件将会被加载和运行在同一进程中，如果没有设置，将会使用默认的进程。
- **exception_handler** 是当进程发生异常是的处理方法。值可以是`exit`或者`respawn`，为空则不进行处理。
  - exit ,当异常发生时整个进程需要停止运行。
  - respawn，当异常发生时，进程需要重启。

-----

### 2.7 Timer

Timer可用于创建定时任务，以定期运行或只运行一次。

#### 2.7.1 Timer Interface

```c++
 /**
   * @brief Construct a new Timer object
   *
   * @param period The period of the timer, unit is ms
   * @param callback The tasks that the timer needs to perform
   * @param oneshot True: perform the callback only after the first timing cycle
   *                False: perform the callback every timed period
   */
  Timer(uint32_t period, std::function<void()> callback, bool oneshot);
```

另一种构造方法是，先封装Timer的配置参数，然后调用构造函数：
period对应定时器周期，单位为ms,范围(1~512*64ms),
callback:对应定时器的回调函数，即周期执行的任务
oneshot:是否只执行一次

```c++
struct TimerOption {
  uint32_t period;                 // The period of the timer, unit is ms
  std::function<void()> callback;  // The tasks that the timer needs to perform
  bool oneshot;  // True: perform the callback only after the first timing cycle
                 // False: perform the callback every timed period
};
/**
 * @brief Construct a new Timer object
 *
 * @param opt Timer option
 */
explicit Timer(TimerOption opt);
```

#### 2.7.2 Start Timer

在创建一个Timer实例后，通过`Timer::Start()`来开启定时器

#### 2.7.3 Stop Timer

通过`Timer::Stop()`来手动停止一个已经开启的定时器

**Demo：**

```c++
#include <iostream>
#include "cyber/cyber.h"
int main(int argc,char** argv){
    cyber::Init(argv[0]);
    cyber::Timer timer(100,[](){
        std::cout << cyber::Time::Now() << std::endl;
    }，false);
    timer.Start();
    sleep(1); //单位s
    Timer.Stop();
}
```

-----

### 2.8 Time API

Time是用于管理时间的类（**注意不不同于Timer**），可以用于当前时间的获取，耗时的计算，时间转换等等。
Time类接口如下：

```c++
// constructor, passing in a different value to construct Time
Time(uint64_t nanoseconds); //uint64_t, in nanoseconds
Time(int nanoseconds); // int type, unit: nanoseconds
Time(double seconds); // double, in seconds
Time(uint32_t seconds, uint32_t nanoseconds);
// seconds seconds + nanoseconds nanoseconds
Static Time Now(); // Get the current time
Double ToSecond() const; // convert to seconds
Uint64_t ToNanosecond() const; // Convert to nanoseconds
Std::string ToString() const; // Convert to a string in the format "2018-07-10 20:21:51.123456789"
Bool IsZero() const; // Determine if the time is 0
```

**Demo:**

```c++
#include <iostream>
#include "cyber/cyber.h"
#include "cyber/duration.h"
int main(int argc, char** argv){
	cyber::Init(argv[0]);
    Time t1(1531225311123456789UL);
    std::cout << t1.ToString() << std::endl;// 2018-07-10 20:21:51.123456789
    //duration time interval
    Time t1(100);
    Duration d(200);
    Time t2(300);
    assert(d == (t2-t1)); //true
}
```

-----

### 2.9 Record flie:Read and Write

#### 2.9.1 Reading the Reader file

**RecordReader**用于组件在Cyber框架下读取消息。每一个RecordReader可以打开一个已经存在的record文件通过该类的`open`方法，线程将异步读取recode文件中的信息。用户仅需要执行`ReadMessage`提取`RecordReader`中的最新信息，然后通过`GetCurrentMessageChannelName`,`GetCurrentRawMessage`,`GetCurrentMessageTime`来获取信息。
**RecordWriter**用于组件在Cyber框架下记录信息。每一个RecordWriter可以创建一个新的record文件通过`Open`方法。用户通过`WriteMessage`和`WriteChannel`来将`message`和`channel`信息写入到record中。

### 2.10 C++ API Directory



## 3. CyberRT提供的工具

Apollo CyberRT提供了一个可视化工具`cyber_visualizer`和两个命令行工具`cyber_monitor`和`cyber_recorder`
注意：使用工具需要使用apollo docker环境，同时由于所有来自CyberRT的工具都依赖于CyberRT库，因此，在使
用CyberRT的工具之前，需要使用`setup.bash`为环境设置源文件，如下所示：

```bash
source /your-path-to-apollo-install-dir/cyber/setup.bash
```

**Cyber_visualizer:**
运行：

```bash
cyber_visualizer
```

交互：
当数据在Cyber RT中通过通道时，所有通道的列表显示在通道名称下，可以通过Cyber RT的record tool(`cyber_recorder`)从另一个终端重放数据，则`cyber_visualizer`将接收所有活动通道的信息（来自重放数据）并显示出来。通过单击工具栏中的选项，可以启用参考网格，显示点云，添加图像，或同时显示多个相机的数据。

**Cyber_monitor：**
命令行工具`cyber_monitor`提供了一个清晰的视图，显示终端中的Apollo Cyber RT实时通道列表信息。

```bash
cyber_monitor
```

通过`-h`选项可以获得`cyber_monitor`的帮助信息。

```bash
cyber_monitor -h
```

通过`-c`选项，可以指定`cyber_monitor`的监视通道，：

```bash
cyber_monitor -c ChannelName
```

![](apollo-cyber_bash\cyber_monitor.png)

启动命令行工具后，显示通道名称和通道的数据类型，默认显示为红色，若有数据流经某通道，则通道的相应行显示为绿色。

**Cyber_recorder:**
`cyber_recorder`是Apollo Cyber RT提供的一种录制/回放工具，它提供了许多有用的功能，包括录制，回放，分割，检查录制文件。

```bash
$ cyber_recorder
usage: cyber_recorder <command>> [<args>]
The cyber_recorder commands are:
info                               Show information of an exist record.
play                               Play an exist record.
record                             Record same topic.
split                              Split an exist record.
recover                            Recover an exist record.
```

同时，Apollo提供了**Rosbag_to_record**将rosbag转换为Apollo Cyber rt提供的记录器文件：

```bash
$ rosbag_to_record
Usage:
rosbag_to_record input.bag output.record
```



## 参考资料

Apollo技术文档

https://github.com/ApolloAuto/apollo/tree/master/docs/cyber

https://github.com/ApolloAuto/apollo/blob/master/cyber/README.md

https://mp.weixin.qq.com/s?__biz=MzI1NjkxOTMyNQ==&mid=2247489447&idx=1&sn=23269b689ce753dd541ea95309806d67&chksm=ea1e05d5dd698cc30f044e3496a2d6e35b3fc674fb5ed56a2dd2da7372268abd2eb0b23df173&mpshare=1&scene=1&srcid=0323lDwCkxxrmpZFEo8U6gZk&sharer_sharetime=1584973075048&sharer_shareid=65219e3b4371d83c28538a4abcff3ebf&key=fee875bb7eca81fa43f2e06e9e7810d275a43a15d157bba99821fd3b9c932a98326c26ca8fee73c2d1c4f10cc1a6d3d374d349f695d11f1a66ce172b7899d78e5ba756bbc353a3067fc471e155a455a5&ascene=1&uin=MjU0MTcyMzYzNw%3D%3D&devicetype=Windows+10&version=62080079&lang=zh_CN&exportkey=A6kdZxOOUE6g8mo3vOd%2BUcM%3D&pass_ticket=oa8DHbLwjRZM4plOFN%2FM7RSGCCQVqNI9RMXwkHnIZDXY%2FJDdElalj3amy8OEJd3j

Apollo社区：

https://mp.weixin.qq.com/s/GEmCpKv1S5wSgJFIhr9PgQ

github:

https://github.com/daohu527/Dig-into-Apollo

知乎：

https://zhuanlan.zhihu.com/p/115046708