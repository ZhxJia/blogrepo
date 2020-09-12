---
title: Apollo中can卡配置
categories:
- CUDA
tags:
- CUDA
mathjax: true
---



CUDA编程基础知识

<!--more-->

## CUDA程序层次结构

特点：

- GPU上有很多并行化的轻量级线程
- kernel在device上执行时实际上时启动很多线程，一个kernel所启动的所有线程称为一个网格(grid)
- 同一个网格上的线程共享相同的全局内存空间，grid是线程结构的第一层次
- 网格又可以分为许多线程块(block),一个线程块里面有很多线程，这是第二个层次
- wrap：32个线程为一组，这是第三个层次。

其中grid和block都是定义为dim3类型的变量，dim3可以视为包含三个无符号整数(x,y,z)成员的结构变量，在定义时，缺省值为1，因此grid和block可以灵活的定义为1-dim,2-dim,3-dim。

定义的grid和block如下所示，kernel在调用时也必须通过执行配置的`<<<grid,block>>>`来指定kernel所使用的线程数及结构，需要注意的是不同的GPU架构，grid和block的维度有限制。

```c++
dim3 grid(3,2);//grid含有3*2个block
dim3 block(5,3);//每个块有5*3个线程 总共有6*15=90个线程
kernel_func<<<grid,block>>>(prams...);

dim3 grid(100,120,32);
dim3 block(16,16,4);
kernel_func<<<grid,block>>>(prams...);
```

## CUDA内置变量

- 一个线程需要两个内置的坐标变量（`blockIdx,threadIdx`）来唯一标识，它们都是dim3类型变量，其中blockIdx指明线程所在的grid中的位置，而threadIdx指明线程所在的block中的位置：
- 其中`threadIdx`包含三个值：`threadIdx.x,threadIdx.y,threadIdx.z`
- `blockIdx`同样包含三个值：`blockIdx.x,blockIdx.y,blockIdx.z`

逻辑循序：x>y>z(按照一维排列)

```c++
dim3 grid(3,2);
dim3 block(5,3);
```

则block的一维排列为：(0,0),(1,0),(2,0),(0,1),(1,1),(2,1)
每个block中thread的排列为:(0,0),(1,0),(2,0),(3,0),(4,0),(0,1),(1,1),(2,1),(3,1),(4,1),(0,2),(1,2),(2,2),(3,2),(4,2) 即先x变

根据这个排列方式，可得全局一维索引。



## GPU内存模型

- 每个线程有自己的私有本地内存（local memory）
- 每个线程块(block)又包含共享内存(shared memory)，可以被线程块所有线程共享，其生命周期与线程块一致。
- 所有的线程都可以访问全局内存(Global Memory).
- 访问一些制度内存块：常量内存(Constant Memory)和纹理内存(Texture Memory)
- L1 cache ,L2 cache

> 全局内存空间很大（就是所谓的显存），但访问速度很慢，而共享内存访问速度很快。

在kernel核函数中使用如下修饰符的内存，称为共享内存： `__shared__`
可以被块`block`中所有的线程访问,使用共享内存时需要注意，不要因为过度使用共享内存而导致SM上活跃的线程束减少，一个线程块所使用的共享内存过多，导致更多的线程块没有办法被SM（shared memory）启动，影响活跃的线程束数量。
共享内存在核函数中声明，生命周期和线程块一致，线程块运行开始，此块的共享内存被分配，当此块结束，则共享内存被释放。

共享内存由于块内所有线程可见，所以存在竞争的问题，为了避免内存竞争，可以通过使用同步语句：

```c++
void __syncthreads();
```

语句相当于在线程块执行时各个线程的一个障碍点，当块内所有线程都执行到本障碍点的时候才能进行下一步的计算，显然频繁使用该函数会影响内核的执行效率。



## 内存使用

- CUDA程序会使用GPU内存和CPU内存
- GPU上内存设计分配和释放，使用CUDA提供的库函数实现
- CUDA/GPU内存与CPU内存的互相传输


### 全局内存和共享内存的管理



