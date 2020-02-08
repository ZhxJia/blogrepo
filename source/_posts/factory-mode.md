---
title: Apollo工厂模式解析
categories:
- 无人驾驶
- c++
tags:
- apollo
- c++
mathjax: true
---
apollo 中工厂模式相关知识介绍
<!-- more -->

本文主要参考：[https://blog.csdn.net/davidhopper/article/details/79197075](https://blog.csdn.net/davidhopper/article/details/79197075)

工厂模式的定义和实现相关资料可参考：

> - [https://zh.wikipedia.org/wiki/%E5%B7%A5%E5%8E%82%E6%96%B9%E6%B3%95](https://zh.wikipedia.org/wiki/工厂方法)
> - 《设计模式：可复用面向对象软件的基础》

Apollo项目中对象的创建，大多使用直接法，例如：

> //在栈(stack)上直接创建对象
>
> ADCTrajectory not_ready_pb;
>
> //在堆(heap)上直接创建对象
>
> ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
>
> > 堆和栈的主要区别在于 `生命周期` 和 `性能` 由于栈的特性，栈上的对象不需要手动管理内存，而堆由程序员自行负责何时用delete释放内存，动态内存的生命周期由我们决定更加灵活。

还有部分通过`单例模式`创建：`DECLARE_SINGLETON(CanClientFactory)` ,其定义如下：

```c++
#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      CallShutdown(instance);                                             \
    }                                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)

```

其中内嵌宏 `DISALLOW_COPY_AND_ASSIGN(classname)`的定义如下：

```c++
#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;
```

因此：`DECLARE_SINGLETON(CanClientFactory)` 展开后的定义为：

```c++
#define DECLARE_SINGLETON(CanClientFactory)                                      \
 public:                                                                  \
  static CanClientFactory *Instance(bool create_if_needed = true) {              \
    static CanClientFactory *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) CanClientFactory(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      CallShutdown(instance);                                             \
    }                                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  CanClientFactory();                                                            \
  CanClientFactory(const CanClientFactory &) = delete;    \
  CanClientFactory &operator=(const CanClientFactory &) = delete;

```

上述代码的意义，首先定义一个静态公有函数Instance(),该函数在栈上创建一个`CanClientFactory` 类的静态对象，然后返回该对象指针。同时，将`CanClientFactory` 类的默认构造函数、复制（或称拷贝）构造函数、复制赋值运算符（或称操作符）定义为私有（private）函数，即禁止进行隐式类型转换和复制操作。

> c++中的static关键词可以用于修改局部变量，函数，类的数据成员以及对象。
>
> ​	静态局部变量只初始化一次，然后每次函数调用时保持其值。
>
> ​	静态成员函数可以直接用类来调用，不需要创建实例来调用。
>
> 静态对象：`static Test t1；` 
>
> ​	静态对象只初始化一次，并且在整个程序的生命周期中都存在，静态对象保存在静态存储区，在程序结束时销毁。
>
> 

