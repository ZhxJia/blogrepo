---

title: bazel C++编译入门
categories: 
- c++
- 工具
tags:
- 工具
---

<!-- more -->

## 安装

参考：https://docs.bazel.build/versions/master/install.html

apt 安装失败，通过binary安装,安装文件下载：https://github.com/bazelbuild/bazel/releases

```bash
sudo apt install g++ unzip zip
# Ubuntu 16.04 (LTS) uses OpenJDK 8 by default:
sudo apt-get install openjdk-8-jdk

# Ubuntu 18.04 (LTS) uses OpenJDK 11 by default:
sudo apt-get install openjdk-11-jdk
sudo bash bazel-<version>-installer-linux-x86_64.sh --user

vim ~/.bashrc
export PATH="$PATH:$HOME/bin"
```

## 编译

参考教程:https://docs.bazel.build/versions/master/tutorial/cpp.html

### 工作区(workspace)

bazel基于工作区(workspace)的概念编译，工作区存放了所有的源代码和Bazel编译输出文件的目录，也就是整个项目的根目录，与bazel编译相关的文件:

- WORKSPACE文件，用于指定当前文件夹就是一个Bazel的工作区，所以WORKSPACE文件总是存在于项目的根目录下。
- 一个或多个BUILD文件，用于告诉Bazel怎么构建项目的不同部分。(如果工作区中的一个目录包含含BUILD文件，即为一个package)

如果要指定一个目录为Bazel工作区，只要在该目录下创建一个空的WORKSPACE文件即可。当Bazel编译项目是，所有的输入和依赖想都必须在同一个工作区中，不同的工作区的文件，除非linked否则彼此独立。

WORKSPACE文件中可以设置第三方代码库，即外部引用

### BUILD文件组成

BUILD文件中最重要的指令是编译指令，告诉Bazel如何编译输出，比如是生成可执行二进制文件还是链接库。BUILD文件中每一天编译指令被称为一个target，它指向一系列的源文件和依赖，一个target也可以指向别的target。

bazelbuild的简单例子可以参考：https://github.com/bazelbuild/examples

```cmake
load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
	name = "hello-world",
	srcs = ["hello-world.cc"],
)
```

`name`即为该target名称，`src`包含了源文件，`cc_binary`为内置的生成二进制文件的指令。

```bash
bazel build //main:hello-world
```

在WORSPACE目录下上述命令行指令即可实现编译，`main:hello-world`表示位于main目录下的target `helo-world`,同时获得如下输出，生成的二进制文件位于`bazel-bin/main/hello-world`
**路径相对于WORKSPACE**

```bash
INFO: Found 1 target...
Target //main:hello-world up-to-date:
  bazel-bin/main/hello-world
```

### 多个target同时编译

对于大型项目来说，一般会把它拆分成多个`target`和多个`package`来实现快速增量的编译。

```cmake
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")                   

cc_library(
   name = "hello-greet",
   srcs = ["hello-greet.cc"],
   hdrs = ["hello-greet.h"],
)

cc_binary(
    name = "hello-world",
    srcs = ["hello-world.cc"],
    deps = [
        ":hello-greet",
    ],
)
```

在这个BUILD文件中，首先编译了`hello-greet`这个库(利用bazel内置的cc_library编译指令)，然后在编译`hello-world`这个二进制文件。`hellow-world`这个target的`deps`属性告诉Bazel，要构建`hello-world`这个二进制文件，首先需要`hello-greet`这个库。

### 多个package同时编译

```
└── stage3
    ├── lib
    │   ├── BUILD
    │   ├── hello-time.cc
    │   └── hello-time.h
    ├── main
    │   ├── BUILD
    │   ├── hello-greet.cc
    │   ├── hello-greet.h
    │   └── hello-world.cc
    ├── README.md
    └── WORKSPACE
```

两个BUILD对应的为两个package,因此对于Bazel来说，整个工作区包含了两个package: lib和main。两个目录下的`BUILD`文件分别定义如下：
main下的BUILD文件：

```cmake
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")                   

cc_library(
    name = "hello-greet",
    srcs = ["hello-greet.cc"],
    hdrs = ["hello-greet.h"],
)

cc_binary(
    name = "hello-world",
    srcs = ["hello-world.cc"],
    deps = [
        ":hello-greet",
        "//lib:hello-time",
    ],
)
```

`lib/BUILD`:

```cmake
load("@rules_cc//cc:defs.bzl", "cc_library")                                

cc_library(
    name = "hello-time",
    srcs = ["hello-time.cc"],
    hdrs = ["hello-time.h"],
    visibility = ["//main:__pkg__"],
)
```

显然main下的`hello-world`这个target依赖于`lib`这个package中的`hello-time`target 即(`deps = ["//lib:hello-time"]`),Bazel通过`deps`这个属性知道自己的依赖项。同时注意到`lib/BUILD`文件中的将`hello-time`这个target显示可见(通过`visibility`属性)，这是因为默认情况下targets只对同一个BUILD文件里的其他targets可见。

```bash
bazel build //main:hello-world
```

