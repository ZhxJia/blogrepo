---
title: py-motmetric使用
tags:
- MOT
categories:
- 目标跟踪
- 多目标跟踪
mathjax: True
---

记录pymotmetric的使用方法

<!--more-->

`import motmetrics as mm`

介绍几个主要的功能接口：

### MOTAccumulator创建及更新(填充)

创建`MOTAccumulator`类对象，用于一个跟踪序列中frame by frame的计算（注：一个数据集可能有多个序列）

```
acc = mm.MOTAccumulator(auto_id=True)
```

然后每一帧往`accumulator`中填充跟踪结果的相关信息,每一帧都有`ground truth/hypotheses id `的列表和它们两两之间的距离`dists`矩阵，例如:
oids(真值):[1,2]			 //ground truth objects id list in this frame
hids(估计):[1,2,3]		  //detector hypotheses id list in this frame
dists:[[0.1, np.nan,0.3], //distance from oid 1 to hid1,2,3
		  [0.5,   0.2   ,0.3]] //distance from oid 2 to hid1,2,3
`np.nan`表示object 1 不能与 hypothesis 2 配对。

```python
'''
Params
------
oids : N array
    Array of object ids.
hids : M array
    Array of hypothesis ids.
dists: NxM array
    Distance matrix. np.nan values to signal do-not-pair constellations.
    See `distances` module for support methods.
@ optional :
frame id (optional when MOTAccumulator.auto_id is specified)
vf: file to log details
-----
Returns
-----
frame_events:pd.DataFrame(containing generated events)
'''
acc.update(oids,hids,dists,frameid,vf)
```

上述update过程通过`the CLEAR MOT merics`中的相关指标，判断`Match,SWITCH,MISS,FP`,想要查看
`accumulator`中发生的事件，通过打印对应`accumulator`的events即可,event通过pandas.Dataframe的格式输出。

```python
print(acc.events) # a pandas DataFrame containing all events
'''
                Type  OId  HId    D
FrameId Event                      
0       0        RAW  NaN  NaN  NaN
        1        RAW  1.0  1.0  0.1
        2        RAW  1.0  3.0  0.3
        3        RAW  2.0  1.0  0.5
        4        RAW  2.0  2.0  0.2
        5        RAW  2.0  3.0  0.3
        6      MATCH  1.0  1.0  0.1
        7      MATCH  2.0  2.0  0.2
        8         FP  NaN  3.0  NaN
'''
```

其中`RAW`表示原始数据(oid,hid,dist),关于mot匹配的事件为oid1和hid1以距离0.1`MATCH`,oid2和hid2以距离0.2`MATCH`,同时由于hid3没有oid与之匹配,判断为`FP`，当然如果仅仅只是想打印mot匹配的相关事件，不想要原始匹配信息，可通过`mot_events`

```python
print(acc.mot_events) # a pandas DataFrame containing MOT only events

"""
                Type  OId HId    D
FrameId Event
0       6      MATCH    1   1  0.1
        7      MATCH    2   2  0.2
        8         FP  NaN   3  NaN
"""
```

通过每一帧一次update的填充更新，最终得到整个数据集各帧的所有`events`。
可能发生的事件有：

```python
'''
Each event type is one of the following
        - `'MATCH'` a match between a object and hypothesis was found
        - `'SWITCH'` a match between a object and hypothesis was found but differs from previous assignment (hypothesisid != previous)
        - `'MISS'` no match for an object was found
        - `'FP'` no match for an hypothesis was found (spurious detections)
        - `'RAW'` events corresponding to raw input
        - `'TRANSFER'` a match between a object and hypothesis was found but differs from previous assignment (objectid != previous)
        - `'ASCEND'` a match between a object and hypothesis was found but differs from previous assignment  (hypothesisid is new)
        - `'MIGRATE'` a match between a object and hypothesis was found but differs from previous assignment  (objectid is new)
'''
```



### 2.Metric的创建计算

根据已经填充的`MOTAccumulator`,创建`metrics`计算并显示指标。

```python
mh = mm.metrics.create()
summary = mh.compute(acc, metrics=['num_frames', 'mota', 'motp'], name='acc')
'''
compute(self, df, ana=None, metrics=None, return_dataframe=True, return_cached=False, name=None)

@brief
Compute metrics on the dataframe / accumulator
Params
------
df : MOTAccumulator or pandas.DataFrame

Kwargs(optional)
------
ana(dict) :cache results for fast computation
metrics(string,list of string or None):the identifiers of the metrics(想要计算的指标的标识符，none全计算)
return_dataframe(bool):return the results as pandas.DataFrame
return_cached(bool):return metrics
name(string):when return a pandas.DataFrame this is the index of the row containing the computed metric valuse
'''
```

通过`print(summary)`，我们可以得到我们给定的`metrics`列表中对应的指标计算结果。

```
     num_frames  mota  motp
acc           3   0.5  0.34
```

上面是计算单个`accumulator`，如果要同时计算多个数据集的指标，则通过`mh.compute_many()`进行计算：

```python
summary = mh.compute_many(
    [acc, acc.events.loc[0:1]],
    metrics=['num_frames', 'mota', 'motp'],
    names=['full', 'part'])
print(summary)

'''
def compute_many(self, dfs, anas=None, metrics=None, names=None, generate_overall=False):

@brief:
Compute metrics on multiple dataframe / accumulators.

Params
------
dfs(list)： list of MOTAccumulator or list of pandas.DataFrame

Kwargs
------
names(list of string): the names of individual rows in the resulting dataframe. 
...其余与compute类似，省略
Returns
-------
df : pandas.DataFrame
'''
```

通过`print(summary)`,可得到各`accumulator`的指标计算结果的显示,`full,part`对应`names`，行名称

```c++
      num_frames  mota      motp
full           3   0.5  0.340000
part           2   0.5  0.166667
```



### 3.格式化输出最终结果

```python
strsummary = mm.io.render_summary(
    summary,
    formatters={'mota' : '{:.2%}'.format},
    namemap={'mota': 'MOTA', 'motp' : 'MOTP'}
)
print(strsummary)
'''
def render_summary(summary, formatters=None, namemap=None, buf=None):

@brief:
Render metrics summary to console friendly tabular output

Params
------
summary :pd.DataFrame Dataframe containing summaries in rows(compute_many/compute的返回值)

Kwargs
------
buf : StringIO-like Buffer to writer to 
formatters(dict): define custorm formatters for individual metrics.you can get preset formatters from
				MetricsHost.formatters
namemap(dict): Dictornary defining new metric names for display I.e`{'num_false_positives': 'FP'}`

Returns
-------
string : Formatted string
'''
```

通过打印`print(strsummary)`可打印指标计算结果，并于未格式化之前进行比较：

```python
# unformatted
     num_frames  mota      motp
full           3   0.5  0.340000
part           2   0.5  0.166667
# formatted
	 num_frames   MOTA      MOTP
full           3 50.00%  0.340000
part           2 50.00%  0.166667
```

pymotmetric中包含了预定义的输出格式，其中metrics即我们想要衡量的计算指标直接使用预设的`motchallenge_metrics`
输出格式使用预定义的`mh.formatters`,指标名称映射使用`mm.io.motchallenge_metric_names`

```
summary = mh.compute_many(
    [acc, acc.events.loc[0:1]],
    metrics=mm.metrics.motchallenge_metrics,
    names=['full', 'part'])

strsummary = mm.io.render_summary(
    summary,
    formatters=mh.formatters,
    namemap=mm.io.motchallenge_metric_names
)
print(strsummary)
```

可得到预定义的输出格式：

```python
      IDF1   IDP   IDR  Rcll  Prcn GT MT PT ML FP FN IDs  FM  MOTA  MOTP IDt IDa IDm
full 83.3% 83.3% 83.3% 83.3% 83.3%  2  1  1  0  1  1   1   1 50.0% 0.340   0   1   0
part 75.0% 75.0% 75.0% 75.0% 75.0%  2  1  1  0  1  1   0   0 50.0% 0.167   0   0   0
```

如果先要获取各个数据集(在MOT中可能是一个数据集的各个sequence)最终的总体指标,将`mh.compute()`或
`mh.compute_many()`中的参数`generate_overall=True`,则在计算完各个序列的指标后，最后会计算一个总的overall指标：

```python
summary = mh.compute_many(
    [acc, acc.events.loc[0:1]],
    metrics=mm.metrics.motchallenge_metrics,
    names=['full', 'part'],
    generate_overall=True                 ##计算整个数据集的指标
    )

strsummary = mm.io.render_summary(
    summary,
    formatters=mh.formatters,
    namemap=mm.io.motchallenge_metric_names
)
print(strsummary)

"""
         IDF1   IDP   IDR  Rcll  Prcn GT MT PT ML FP FN IDs  FM  MOTA  MOTP
full    83.3% 83.3% 83.3% 83.3% 83.3%  2  1  1  0  1  1   1   1 50.0% 0.340
part    75.0% 75.0% 75.0% 75.0% 75.0%  2  1  1  0  1  1   0   0 50.0% 0.167
OVERALL 80.0% 80.0% 80.0% 80.0% 80.0%  4  2  2  0  2  2   1   1 50.0% 0.275
"""
```



### 获取update中的distance距离矩阵

`pymotmetric`中提供的工具为`motmetrics.distance`模块

**Euclidean norm squared on points** 欧氏距离的平方

```python
# Object related points
o = np.array([
    [1., 2],
    [2., 2],
    [3., 2],
])

# Hypothesis related points
h = np.array([
    [0., 0],
    [1., 1],
])

C = mm.distances.norm2squared_matrix(o, h, max_d2=5.)

"""
[[  5.   1.]
 [ nan   2.]
 [ nan   5.]]
"""
```

`mm.distance.norm2squared_matrix(objs,hyps,max_d2=float('inf'))` ,`max_d2`为距离阈值，超过此距离则不进行匹配，设为`np.nan` ,`objs`和`hyps`为列数同一维度的坐标点(ie. 都是二维点)，但是点的数量可以不一致

```python
    """Computes the squared Euclidean distance matrix between object and hypothesis points.

    Params
    ------
    objs : NxM array
        Object points of dim M in rows
    hyps : KxM array
        Hypothesis points of dim M in rows

    Kwargs
    ------
    max_d2 : float
        Maximum tolerable squared Euclidean distance. Object / hypothesis points
        with larger distance are set to np.nan signalling do-not-pair. Defaults
        to +inf

    Returns
    -------
    C : NxK array
        Distance matrix containing pairwise distances or np.nan.
    """
```

**Intersection over union norm for 2D rectangles** :2D框的交并比作为距离衡量：

```
a = np.array([
    [0, 0, 1, 2],    # Format X, Y, Width, Height
    [0, 0, 0.8, 1.5],
])

b = np.array([
    [0, 0, 1, 2],
    [0, 0, 1, 1],
    [0.1, 0.2, 2, 2],
])
mm.distances.iou_matrix(a, b, max_iou=0.5)

"""
[[ 0.          0.5                nan]
 [ 0.4         0.42857143         nan]]
"""
```

`mm.distance.iou_matrix(objs,hyps,max_iou=1.)`
计算iou距离=1-iou，输入格式要求为(x,y,w,h),其中`max_iou`,当计算的iou距离大于`max_iou`，距离置为`np.nan`不进行匹配

```python
    """Computes 'intersection over union (IoU)' distance matrix between object and hypothesis rectangles.

    The IoU is computed as

        IoU(a,b) = 1. - isect(a, b) / union(a, b)

    where isect(a,b) is the area of intersection of two rectangles and union(a, b) the area of union. The
    IoU is bounded between zero and one. 0 when the rectangles overlap perfectly and 1 when the overlap is
    zero.

    Params
    ------
    objs : Nx4 array
        Object rectangles (x,y,w,h) in rows
    hyps : Kx4 array
        Hypothesis rectangles (x,y,w,h) in rows

    Kwargs
    ------
    max_iou : float
        Maximum tolerable overlap distance. Object / hypothesis points
        with larger distance are set to np.nan signalling do-not-pair. Defaults
        to 0.5

    Returns
    -------
    C : NxK array
        Distance matrix containing pairwise distances or np.nan.
    """
```



### 直接利用mot数据集格式的检测结果进行衡量

