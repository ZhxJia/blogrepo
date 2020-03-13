---
{}
title:
date:
updated:
permalink:
categories:
tags:
mathjax:
description:
keywords:
comments:
link:
top:
---





glog cmake 编译的问题

glog 原生不支持cmake，将`FindGlog.cmake`添加到CMake的模块目录下

http://www.yeolar.com/note/2014/12/20/glog/

http://senlinzhan.github.io/2017/10/07/glog/

https://blog.csdn.net/dbzhang800/article/details/6329314

如果gflags也是从包管理器安装的，则使用cmake 也需要findGFlags.cmake

http://www.yeolar.com/note/2014/12/14/gflags/

glog学习

https://izualzhy.cn/glog

protobuf可能遇到的问题

https://111qqz.github.io/2018/04/protobuf-notes/





google 开源风格

https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/scoping/

https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/headers/

命名空间相互污染

using namespace



glog的使用

https://www.cnblogs.com/JZ-Ser/articles/7909800.html

gtest的使用

https://www.cnblogs.com/coderzh/archive/2009/04/06/1426755.html

blob

原caffe中的blob不支持bool和uint8_t类型的数据

https://cloud.tencent.com/developer/article/1394880



并发编程

https://chenxiaowei.gitbook.io/cpp_concurrency_in_action/1.0-chinese

计算confidence处存在的问题

```
            struct CmpByValue {
                bool operator()(const PAIR &lhs, const PAIR &rhs) {
                    return lhs.second <= rhs.second;
                }
            };
```

NMS排反了.





### blob 中的offset 与opencv中通过Mat.ptr(n)快速实现各行像素元素的访问