---

title: DS(Dempster-Shafer)多元信息融合证据理论
categories:
- autonomous
- apollo
tags:
- apollo
- sensor fusion
mathjax: true
---

70年代初，Dempster和Shafer建立的一套数学理论，称为D-S证据理论，是贝叶斯推理方法的推广，广泛应用于多源信息融合，不确定性推理。

贝叶斯理论通过已知的三个概率推断第四个概率，即在某些已知条件下，推断某事件发生的概率，
$$
P(A|B)=\frac{P(B|A)P(A)}{P(B)}\\
$$
P(A|B)为已知B发生情况下A的后验概率
P(A)时A的先验概率(边缘概率)
P(B|A)为以子A发生的情况下B的后验概率
P(B)是B的先验概率，通常表示为$P(B)=\sum_{j}P(B|A_j)P(A_j),A_j为事件几何的各子集$
也可表述为:**后验概率 = (似然性*先验概率)/标准化常量**
所以A发生的后验概率与A的先验概率和似然度的乘积成正比。

是否有不需要先验概率直接获得后验概率的方法呢？证据的本质就是基于观测对不同的假设赋予权值的一种方法：
(1) 能够处理任意数量的假设
(2) 能够把证据的权值解释为一种函数，而这个函数把假设的先验概率空间映射到基于观测的假设后验概率空间。

<!--more-->

### 1. DS证据理论的基本概念--mass函数、信度函数、似真度函数

**定义1：基本概率分配（BPA）**

> DS证据理论设空间$H$表示某个有限集合，称为假设空间；假设$P(H)$表示$H$所有子集(若样本空间有N个元素，则所有的子集有$2^N$个)构成的集类(称为H的幂集)，则映射$m:P(H){\rightarrow}[0,1]$称为一个基本概率赋值(BPA)或`mass`函数，需满足下列条件：
> (1)  $m(\phi)=0$ 
> (2)  $\sum_{A\subset H}m(A)=1$（假设空间所有子集的mass之和为1）时
> 则`mass`函数实际上是对各种假设的评价权值。注意基本概率赋值不是概率，不满足可列可加性。

实际就是分配给H的每一个子集一个实数，且空集的基本概率数为0，所有子集的概率分配数之和为1，这个实数位于[0,1]之间，同时这个映射关系不是唯一的且概率分配函数与概率是不同的。

设样本空间$H=\{红，黄，蓝\}$
M({红})=0.3 (认为是红色的信任度为0.3)，M({黄})=0，M({蓝})=0.1，M({红，黄})=0.2，M({红，蓝})=0.2(认为是红色或是蓝色的信息度为0.2)，M({黄，蓝})=0.1，M({红，黄，蓝})=0.1(表示命题不知道是红色，黄色还是蓝色，这0.1的权值不知道怎么分配),
M({$\Phi$})=0,但M({红})+M({黄})+M({蓝})=0.4$\neq$1.0  所以概率分配函数与概率不同。

参考韩崇昭老师的《多源信息融合》举个栗子：
两个医生同时给一个病人看病，甲医生认为0.9可能性是感冒，0.1可能性是某种病症；乙医生认为0.2可能性**不是**感冒，0.8可能性是某种病症。此时假设空间位$H=\{h,\bar h\}$ ,h是诊断为感冒，$\bar h$表示诊断为不是感冒。$P(H)=\{\Phi,\{h\},\{\bar h\},H\}$,$\Phi$表示为不可能事件：“既是感冒又不是感冒”，$H$表示事件“可能是感冒，又可能不是感冒”，则构造的mass函数为：
$m_1(h)=0.9$ 表示甲医生认为是感冒的可能性；
$m_1(H)=0.1$ 表示甲医生认为是某种说不清的病症的可能性
$m_2(\bar h)=0.2$表示乙医生认为不是感冒的可能性；
$m_2(H)=0.8$表示乙医生认为是某种说不清的病症的可能性
问题是判断患者是感冒的可能性是多少？
需要注意的是$H=\{h\}\cup\{\bar h\}$,且$\{h\}\cap\{\bar h\}=\Phi$ 但 $m_1(H)\neq m_1(h)+m_1(\bar h)$ 即mass函数不是表示的概率。

**定义2：信度函数(belief function)**

> 设$H$表示某个有限集合，$P(H)$表示$H$的所有子集构成的集类，映射$Bel:P(H)\rightarrow[0,1]$称为信度函数，
> $Bel(A)=\sum_{B\subseteq A }M(B)$,$\forall A\subseteq H$ ,如果满足：
> (1) $Bel(\Phi)=0;Bel(H)=1$ (样本空间的信度函数为1)
> (2) 对$H$中的任意子集$A_1,A_2,\dots,A_n$有：
> $$
> Bel(\bigcup_{i=1}^{n}A_i) \ge \sum_{I\subseteq\{1,2,...,n\}\\I\neq\Phi}(-1)^{|I|+1}Bel(\bigcap_{i\in I}A_i)
> $$
> 其中,$|I|$表示集合$I$中的元素个数。
> 说明信度函数表示对假设命题A为真的信任程度估计的下限(悲观估计)，假定仅对$H$中的任意两个子集$A_1,A_2$有：
> $$
> Bel(A_1 \cup A_2)\ge Bel(A_1)+Bel(A_2)-Bel(A_1\cap A_2)
> $$
> 此时称$Bel:P(H)\rightarrow[0,1]$为弱信度函数(weak belief function)

设样本空间$H=\{红，黄，蓝\}$
M({红})=0.3，M({黄})=0，M({红，黄})=0.2
Bel({红，黄})=M({红})+M({黄})+M({红，黄})=0.3+0+0.2=0.5,即对命题认为是红色或黄色的总信任程度为0.5

**定义3： 似真度函数（plausibility function）**

> 假设$H$表示某个有限集合，$P(H)$表示H的所有子集构成的集类，映射$Pl:P(H)\rightarrow[0,1]$称为似真度函数，如果满足：
> (1)  $Pl(\Phi)=0;Pl(H)=1$
> (2)  对H中的任意子集$A_1,A_2,...,A_n$有
> $$
> Pl(\bigcap_{i=1}^{n}A_i) \le \sum_{I\subseteq\{1,2,...,n\}\\I\neq\Phi}(-1)^{|I|+1}Pl(\bigcup_{i\in I}A_i)
> $$
> 说明似真度函数表示对假设的信任程度估计的上限(乐观估计)。假定仅对H中的任意两个子集$A_1,A_2$有：
> $$
> Pl(A_1 \cap A_2)\le Pl(A_1)+Pl(A_2)-Pl(A_1\cup A_2)
> $$
> 则称$Pl:P(H)\rightarrow[0,1]$为弱似真度函数。

**定理1：**

> $Pl(A)=1-Bel(A^{\lnot}),\forall A\subseteq H$ ，$A \lnot $表示A的补集
> $Bel(A)=1-Pl(A\lnot)$

**定理2：**

> $Bel(A)=\sum_{D\subseteq A}m(D)$
> $Pl(A) =\sum_{D\cap A\neq\Phi}m(D)$

例：设样本空间$H=\{红，黄，蓝\}$
M({红})=0.3，M({黄})=0，M({红，黄})=0.2
Bel({红，黄})=M({红})+M({黄})+M({红，黄})=0.5
Pl({蓝})=1-Bel({蓝}$\lnot$)=1-Bel({红，黄})=1-0.5=0.5 表示

`信度函数`和`似真度函数`表征了一个命题不确定性的上下限，两者之间的关系如下：
因为
$Bel(A)+Bel(\lnot A)=\sum_{B\subseteq A}M(B)+\sum_{C\subseteq\lnot A}M(C)\le\sum_{E\subseteq D}M(E)=1$

所以
$Pl(A)-Bel(A)=1-Bel(\lnot A)-Bel(A)=1-(Bel(\lnot A)+Bel(A))\ge0$

$Pl(A)\ge Bel(A)$
其中$Bel(A):对命题A为真的信任程度$
$Pl(A):对A为非假的信任程度$
$A(Bel(A),Pl(A))$为命题A信任程度的下限和上限，下面给出几种特殊情况：

| 命题A信任度的上下限 | 解释                                  |
| ------------------- | ------------------------------------- |
| A(0,1)              | 表示对命题A一无所知                   |
| A(1,1)              | A为真                                 |
| A(0,0)              | A为假                                 |
| A(a,1),0<a<1        | 对命题A部分信任，信任程度为a          |
| A(0,b),0<b<1        | 对命题$\lnot A $部分信任，信任程度为b |
| A(a,b)              | 表示对命题A和$\lnot A$同时信任        |

设球的颜色样本空间$H=\{红，黄，蓝\}$
M({红})=0.3 ，M({黄})=0，M({蓝})=0.1，M({红，黄})=0.2，M({红，蓝})=0.2，M({黄，蓝})=0.1，M({红，黄，蓝})=0.1
则球是红色命题的信任度的上下限为A(0.3,0.8)

**定理3：**

> 设H是有限集合，Bel和Pl分别是定义在P(H)上的信度函数和似真度函数，$z_1,z_2,...,z_l\in O$为$l$个互斥且完备的观测，即$\mu_(z_i)$表示$z_i$发生的概率，满足$z_i \cap z_j=\Phi,\forall i\neq j$且$\sum_{i=1}^l \mu(z_i)=1$对于每个$z_i \in O$,当$m(\centerdot|z_i),Bel(\centerdot|z_i),Pl(\centerdot|z_i)$分别是H上的mass函数，信度函数，似真度函数时，则：
> $$
> m(A)=\sum_{i=1}^{l}m(A|z_i)\mu(z_i)\\
> Bel(A)=\sum_{i=1}^{l}Bel(A|z_i)\mu(z_i)\\
> m(A)=\sum_{i=1}^{l}Pl(A|z_i)\mu(z_i)
> $$
> 仍分别是H上的mass函数，信度函数和似真度函数。

### 2. Dempster-Shafer证据的组合

意义在于两个不同的可能性判断，经过合成变为统一的判断，需要注意的是这是一种集合运算，与概率的数值计算不同。

**定理4：**

> 设$m_1,m_2$是H上的两个mass函数，则
> $m(\Phi)=0$
> $m(A)=\frac{1}{N}\sum_{E\cap F=A}m_1(E)m_2(F),A\neq \Phi$
> 是mass函数，其中$N=\sum_{E\cap F\neq\Phi}m_1(E)m_2(F)>0$为归一化系数。
> 一般记为$m=m_1\oplus m_2$
> 扩展到一般情况下：$m(A)=(m_1 \oplus...\oplus m_n)=\frac{1}{N}\sum_{\bigcap  _{i=1}^{n}E_i=A}\prod_{i=1}^n m_i(E_i),A\neq \Phi$
> 如果$N=0$，则由于mass函数存在矛盾不能合成。

例：
D={黑，白}，且设
$M_2$({黑}，{白}，{黑，白}，$\Phi$)=(0.6, 0.3, 0.1, 0)
$M_1$({黑}，{白}，{黑，白}，$\Phi$)=(0.3, 0.5, 0.2, 0)
则
$$
\begin{align}
N&=\sum_{E\cap F\neq\Phi}m_1(E)m_2(F)\\
&=1-\sum_{E\cap F=\Phi}m_1(E)m_2(F)\\
&=1-[M_1(\{黑\})*M_2(\{白\})+M_1(\{白\})*M_2(\{黑\})]\\
&=1-[0.3*0.3+0.5*0.6]=0.61\\
 \\
M(\{黑\})&=\frac{1}{0.61}[M_1(\{黑\})*M_2(\{黑\})+M_1(\{黑\})*M_2(\{黑,白\})+M_1(\{黑，白\})*M_2(\{黑\})]\\
&=\frac{1}{0.61}*[0.3*0.6+0.3*0.1+0.2*0.6]=0.54
\end{align}
$$


### 3. 证据推理：

**定理5：**

> 设m是假设空间H上的mass函数， P(H)表示H的所有子集构成的幂集，F是H上的概率分布，则有
> $v:p(H)\rightarrow[0,1]$ ,满足$v(\Phi)=0$，且：
> $$
> v(A)=\frac{m(A)*F(A)}{\sum_{\Phi \neq B\subseteq H}m(B)*F(B)}
> $$
> 以及$\gamma:P(H)\rightarrow[0,,1]$,满足$\gamma(\Phi)=0$,且
> $$
> \gamma(A)=\frac{m(A)/F(A)}{\sum_{\Phi\neq B\subseteq H}m(B)/F(B)}
> $$
> 仍是H上的mass函数。

**定理6：**

> 设H是有限集合，$m_1,m_2$是定义在P(H)上的mass函数，P是H上的概率分布，则有$v:P(H)\rightarrow[0,1]$ ,满足
> $v(\Phi)=0$,且
> $$
> v(A)=\frac{(r_1\oplus r_2)(A)*F(A)}{\sum_{\Phi \neq D\subseteq H}(r_1 \oplus r_2)(D)*F(D)},A\in P(H)
> $$
> 仍是H上的mass函数，其中：
> $$
> \gamma_i(A)=\frac{m_i(A)/F(A)}{\sum_{\Phi\neq B\subseteq H}m_i(B)/F(B)},i=1,2
> $$
> 记为$v(A)=(m_1\otimes m_2)(A)$

证据推理的一般步骤为：

1. 计算mass函数$m_i(A)$,即：
   $m_i(A)=m(A|z_i)\mu_i(z_i)+m(A|\overline{z}_i)\mu_i(\overline{z}_i),i=1,..,l$

2. 利用先验概率F,计算mass函数$\gamma_i(A)$
   $$
   \gamma_i(A)=\frac{m_i(A)/F(A)}{\sum_{\Phi\neq B\subseteq H}m_i(B)/F(B)},i=1,2
   $$
   
3. 利用Dempster-Shafer合成公式，计算mass函数$\gamma(A)$
   $$
   \gamma(A)=(r_1 \oplus r_2 \oplus ... \oplus r_l)(A)
   $$
   
4. 计算mass函数$v(A)$
   $$
   v(A)=\frac{\gamma(A)*F(A)}{\sum_{\Phi \neq D\subseteq H}\gamma(D)*F(D)},A\in P(H)
   $$
   
5. 计算信度函数和似真度函数，
   $$
   Bel(A)=\sum_{D\subseteq A}v(D)\\
   Pl(A) =\sum_{D\cap A\neq\Phi}v(D)
   $$
   得到信任区间$[Bel(A),Pl(A)]$

例子：
设有规则：
(1) 如果流鼻涕则感冒但非过敏性鼻炎(0.9),或过敏性鼻炎但非感冒(0.1)
(2)如果眼发炎则感冒但非过敏性鼻炎(0.8),或过敏性鼻炎但非感冒(0.05)

有事实（先验概率）：
(1) 小王流鼻涕(0.9)
(2) 小王眼发炎(0.4)
问：小王患的啥病？

> 首先取样本空间$H=\{h_1,h_2,h_3\}$
> $h_1$表示感冒但非过敏性鼻炎,$h_2$表示过敏性鼻炎但非感冒，$h_3$表示同时得了两种病
> 取下面的基本概率分配函数：
> $$
> M_1(\{h_1\})=0.9*0.9=0.81\\
> M_1(\{h_2\})=0.9*0.1=0.09\\
> M_1(\{h_1,h_2,h_3\})=1-M_1(\{h_1\})-M_1(\{h_2\})=0.1\\
> M_2(\{h_1\})=0.4*0.8=0.32\\
> M_2(\{h_2\})=0.4*0.051=0.02\\
> M_2(\{h_1,h_2,h_3\})=1-M_2(\{h_1\})-M_2(\{h_2\})=0.66\\
> $$
> 然后进行概率合成：
> $$
> N=1-[M_1(\{h_1\})M_2(\{h_2\})+M_1(\{h_2\})M_2(\{h_1\})]=1-0.81*0.02-0.09*0.32=0.955\\
> M(\{h_1\})=\frac{1}{0.955}*[0.81*0.32+0.81*0.66+0.1*0.32]=0.865\\
> M(\{h_2\})=\frac{1}{0.955}*[0.09*0.02+0.09*0.66+0.1*0.02]=0.066\\
> M(\{h_1,h_2,h_3\})=1-0.865-0.066=0.069
> $$
> 信任度：
> $$
> Bel(\{h_1\})=M(\{h_1\})=0.865\\
> Bel(\{h_2\})=M(\{h_2\})=0.066\\
> $$
> 似真度：
> $$
> Pl(\{h_1\})=M(\{h_1\})+M(\{h_1,h_2,h_3\})=0.934\\
> Pl(\{h_2\})=M(\{h_2\})+M(\{h_1,h_2,h_3\})=0.066+0.069=0.135
> $$
> 则事件$\{h_1\}$的信任区间为[0.865,0.934],而事件$\{h_2\}$的信任区间为[0.066,0.135],因此小王应该是感冒了。





### 4.mass函数的获取方法

1. 考虑目标类型数和环境加权系数确定mass函数
2. 基于统计证据的mass函数获取方法
3. 基于隶属度函数生成mass函数的方法





### 5.证据理论存在的主要问题与发展

### 参考

[CSDN:D-S证据理论学习笔记](https://blog.csdn.net/am45337908/article/details/48832947)