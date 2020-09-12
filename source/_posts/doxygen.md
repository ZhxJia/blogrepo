---

title: doxygen安装
categories:
- tools
- doxygen
tags:
- doxygen
mathjax: true
---

doxygen 自动文档生成工具安装

<!-- more -->

> 参考：https://www.ibm.com/developerworks/cn/aix/library/au-learningdoxygen/index.html

## 安装doxygen

```bash
sudo apt-get install flex bison
git clone https://github.com/doxygen/doxygen.git
cd doxygen
mkdir build
cd build
cmake .. & make 
sudo make install 
```

## 使用Doxygen生成文档

--目前应用好像有困难　　暂时先不写了