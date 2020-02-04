---
title: 最优估计 -- kalman and lsm
tags:
- 优化
categories:
- 最优估计
mathjax: true
---
kalman Filter 和 least square 目的均为最优化某一指标，指标是优化的关键：

<!-- more -->

常用的估计准则有：

- 无偏估计：估计值的期望等于被估计参数的真实值。

- 线性最小方差估计：将估计量限制为观测值的线性函数，已知观测量Z和和被估计量X一二阶矩（EX,Var{X},EZ,Var{Z},Cov{X,Z}）,使估计误差的方差最小，即最小化$tr\{E[\tilde{X}-E\tilde{X}][\tilde{X}-E\tilde{X}]^{T}\}$ ,$\tilde{X}$为估计误差（等价于最小化均方误差阵，若为无偏估计）可得其无偏估计值为$\tilde{X}_{LMV}(Z)=EX+cov(X,Z)(var(Z))^{-1}[Z-EZ]$对于观测模型Z=HX+V，上述条件若已知

  $\{EX=\mu_x,Var(X)=P_x,EV=0,Var(V)=R,E(XV^T)=0\}$ 即可得到。

- 最小二乘估计：对数据（X、Z）的统计特性一无所知，但仍需对X进行估计，目标是最小化残差[^1]平方和。

  满足最小方差必满足残差平方和最小，反之则不成立。



#### 经典最小二乘

针对隐状态X，若其无法直接观测，但间接获取其观测值$Z=[z_1,z_2,\dots,z_n]^T$  ,若其观测值为状态值的线性函数：
$$
Z_i=H_iX+V_i,i=1,\dots,n
$$
$z_i$为第i次测量的观测值，$H_i$为第i次测量的观测模型(设计矩阵，实验的观测值)，$V_i$为第i次测量的噪声（误差）。

则第i次测量的估计误差：
$$
\hat{e_i}=z_i-H_i\hat{X}
$$
则n次测量的误差（残差）平方和为优化指标：
$$
J(\hat{X})=\sum_{i=1}^{n}{(z_i-H_i\hat{X})^2}=(Z-H\hat{X})^T(Z-H\hat{X}) \\
=tr[(Z-H\hat{X})(Z-H\hat{X})^T]
$$
令$\frac{\partial{J}}{\partial{\hat{X}}}=0$ ,可得最小二乘估计值：
$$
\hat{X}_{LS}=(H^TH)^{-1}H^TZ
$$
将$Z=HX+V$此时状态的估计误差：
$$
\tilde{X}_{LS}=X-\hat{X}_{LS}=-(H^TH)^{-1}H^TV
$$
若测量噪声均值为0，则$E(\tilde{X}_{LS})=0$,此时最小二乘估计为**<u>无偏估计</u>**，**状态估计误差的（协）方差[^ 2] $Var(\tilde{X}_{LS})=E[(\tilde{X}-E\tilde{X})(\tilde{X}-E\tilde{X})^T]$与估计量的均方误差矩阵$E[X-\hat{X}][X-\hat{X}]^T$相等**。可见标准最小二乘不需要噪声V的任何统计信息。

由(5)式可得：
$$
\begin{align}
Var(\tilde{X}_{LS})=E[X-\hat{X}][X-\hat{X}]^T & = (H^TH)^{-1}H^TE(VV^T)H(H^TH)^{-1}\\
&=(H^TH)^{-1}H^TRH(H^TH)^{-1}
\end{align}
$$
其中$R=E(VV^T)$为测量误差（噪声）的（协）方差阵。

#### 加权最小二乘（weighted least square）

在经典最小二乘中，假定每一次测量的权重相同，但是一般来说近期数据比远期数据影响更大，因此引入加权最小二乘，其指标形式：
$$
J_W(\hat{X})=\sum_{i=1}^{n}{(z_i-H_i\hat{X})^2}=(Z-H\hat{X})^TW(Z-H\hat{X})
$$
同样使其偏导数为0,可得
$$
\hat{X}_{LSW}=(H^TWH)^{-1}H^TWZ
$$

----------------------

由附录[^3],若噪声不满足同方差，则普通最小二乘(4)并不是BLUE，此时噪声的协方差阵

$E[VV^T]=\sigma^2R,R\neq{I}$ ,$R=\begin{bmatrix}r_1\\&\ddots\\&& r_n\end{bmatrix}$,即原模型存在异方差性。

设$R=DD^T,D=\begin{bmatrix}\sqrt{r_1}\\&\ddots\\&& \sqrt{r_n}\end{bmatrix}$ ,用$D^{-1}$同时左乘$Z=HX+V$两端得到新的模型：
$$
\begin{align}
D^{-1}Z&=D^{-1}HX+D^{-1}V \\
Z^{\star}&=H^{\star}X+V^{\star}
\end{align}
$$
此时,原模型的加权最小二乘估计量为无偏的。
$$
\begin{align}
E[V^{\star}V^{\star T}]&=E[D^{-1}VV^TD^{-1\ T}]\\
&=D^{-1}E[VV^T]D^{-1\ T}\\
&=\sigma^2D^{-1}RD^{-1\ T}\\
&=\sigma^2I
\end{align}
$$
此时得到的参数估计为：
$$
\begin{align}
\hat{X}_{LSW}&=(H^{\star T}H^{\star})^{-1}H^{\star T}Z^{\star}\\
&=(H^TR^{-1}H)^{-1}H^TR^{-1}Z
\end{align}
$$
可以证明（见附录），当$W=R^{-1}$时，最小二乘估计时缺少初值条件下的**<u>线性无偏最小方差估计</u>**（BLUE,Best Linear Unbiased Estimation）——即能够使估计误差的方差阵最小，又称马尔可夫估计,其中
$$
R=E[VV^T]
$$
为随机噪声的（协）方差阵（对称正定阵）。

#### 递推最小二乘（Recursive Least Square,RLS）

上述方法进行一次估计需要所有历史数据，不利于在线估计，考虑前n次测量：
$$
Z_n=H_nX+V_n
$$
则加权的最小二乘估计为：
$$
\hat{X}_{LSW}(n)=(H_{n}^TR_{n}^{-1}H_n)^{-1}H_{n}^TR_{n}^{-1}Z_n
$$
估计误差的（协）方差矩阵为：
$$
\begin{align}
P_n&=E[\tilde{X}_{LSW}(n)\tilde{X}_{LSW}^T(n)]\\
&=E[-(H^TR^{-1}H)^{-1}]H^TR^{-1}VV^TR^{-1}H(H^TR^{-1}H)^{-1}\\
&=(H^TR^{-1}H)^{-1}H^TR^{-1}H(H^TR^{-1}H)^{-1}\\
&=(H^TR^{-1}H)^{-1}
\end{align}
$$
结合上述两式，可得：
$$
\hat{X}_{LSW}(n)=P_nH_{n}^TR_{n}^{-1}Z_n
$$
现得到一个新的测量值：
$$
z_{n+1}=H_{n+1}X+v_{n+1}
$$
添加到矩阵中：
$$
\hat{X}_{LSW}(n+1)=(H_{n+1}^TR_{n+1}^{-1}H_{n+1})^{-1}H_{n+1}^TR_{n+1}^{-1}Z_{n+1}
$$
将<u>新的测量噪声</u>加入到原本的测量噪声矩阵中：R阵应为对角阵：
$$
R_{k+1}^{-1}=
\begin{bmatrix}
R_n^{-1} & 0 \\0&r^{-1}_{n+1}
\end{bmatrix}
$$
将式子展开：
$$
P_{n+1}^{-1}=H_{n+1}^TR_{n+1}^{-1}H_{n+1}=[H_n^T,h_{n+1}^T]
\begin{bmatrix}
R_n^{-1} & 0 \\0&r^{-1}_{n+1}
\end{bmatrix}
\begin{bmatrix}
H_n\\h_{n+1}
\end{bmatrix}
=H_n^TR_n^{-1}H_n+h_{n+1}^Tr_{n+1}^{-1}h_{n+1}
$$
即：
$$
P_{n+1}^{-1}=P_n^{-1}+h_{n+1}^Tr_{n+1}^{-1}h_{n+1}
$$
综上，可以推得：
$$
\begin{align}
P_{n+1}&=P_n-P_nh_{n+1}^T[h_{n+1}P_nh_{n+1}^T+r_{n+1}]^{-1}h_{n+1}P_n\\
K_{n+1} &= P_{n+1}h_{n+1}^Tr_{n+1}^{-1}\\
\hat{X}_{LSW}(n+1)&=\hat{X}_{LSW}(n)+K_{n+1}[z_{n+1}-h_{n+1}\hat{X}_{LSW}(n)]
\end{align}
$$
其中$K_{n+1}$可将(31)代入展开为：
$$
K_{n+1} = P_nh_{n+1}^T[h_{n+1}P_nh_{n+1}^T+r_{n+1}]^{-1}
$$
因此$P_{n+1}$亦可表示为：
$$
P_{n+1}=P_n-K_{n+1}h_{n+1}P_n
$$

#### 卡尔曼滤波

若被估计量X不随时间变化，或随时间缓慢变化则为“静态估计”，而被估计量随时间变化为“动态估计”。









参考：

> https://blog.csdn.net/qinruiyan/article/details/50793114
>
> 《最优估计理论》刘胜，张红梅，科学出版社
>
> [^1]: 残差在数理统计中是指实际观察值和估计值之间的差。若设线性回归模型为$Z=HX+V$ ,其中Z为n维输出向量，H是$n\times(p+1)$ 阶设计矩阵，X是p+1维向量，V为n维随机变量(扰动)。则回归系数的估计值$\hat{X}=(H^TH)^{-1}H^TZ$ ，拟合值$\hat{Z} = H\hat{X}=H(H^TH)^{-1}H^TZ$,残差为$\hat{\epsilon}=z_i-\hat{z_i}=z_i-H_i\hat{X}$ ，其由观测真值和H阵给出，不考虑噪声V。
> [^ 2]: https://zh.wikipedia.org/wiki/%E5%8D%8F%E6%96%B9%E5%B7%AE%E7%9F%A9%E9%98%B5<img src="C:\Users\Jachin Jac\AppData\Roaming\Typora\typora-user-images\image-20191215194027795.png" alt="image-20191215194027795" style="zoom:50%;" />
> [^3]: 在线性回归模型中，如果随机噪声（误差）满足**零均值、同方差且互不相关**，则回归系数的最优线性无偏估计（BLUE，Best Linear unbiased estimator）就是普通最小二乘估计。
>
> ###### 最佳线性无偏估计（GM假设）
>
> 假设多元线性回归模型：$Z=HX+V$
> $$
> \begin{align}Z&=(z_1,\dots,z_n)^T\\H&=\begin{bmatrix}h_{ij}\end{bmatrix}_{n\times{p}}\\X&=(x_o,\dots,x_p)\\V&=(v_0,\dots,v_n)\end{align}
> $$
> 则GM假设：
> $$
> \begin{align}E(V|H)&=0,\forall H\ (零均值)\\Var(V|H)&=E(VV^T|H)=\sigma^2I_n\ (同方差且不相关)\end{align}
> $$
> 则此时对参数X的最佳线性无偏估计为：
> $$
> \hat{X}=(H^TH)^{-1}H^TZ
> $$
>
> ###### 最小二乘估计与最小方差估计等价条件证明：
>
> ![image-20191216221314847](C:\Users\Jachin Jac\AppData\Roaming\Typora\typora-user-images\image-20191216221314847.png)
>
> ###### 各种估计方法的比较：
>
> ![image-20191216221240802](C:\Users\Jachin Jac\AppData\Roaming\Typora\typora-user-images\image-20191216221240802.png)

