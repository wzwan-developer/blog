---
title: "Least-Squares Estimation of Transformation Parameters Between Two Point Patterns"
description: 在相机雷达联合标定算法的论文中，作者使用了该算法用于粗标定，因此打算一探究竟。
date: 2025-01-12T14:48:16+08:00
image: title.png
math: true
license: 
hidden: false
comments: true
draft: false
categories:
- Math
---
## 文章内容
### 摘要
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Abstract*— In many applications of computer vision, the following probler is encountered. Two point patterns （sets of points）$\left\{a_i \right\} $ and $\left\{y_i\right\}$;i = 1,2 $\cdots$,n are given in m-dimensional space, and we want to find the similarity transformation parameters （rotation， translation, and scaling） that give the least mean squared error between these point patterns. Recently Arun *et al.* and Horn *et al.* have presented a solution of this problem. Their solution, however, sometimes fails to give a correct rotation matrix and gives a reflection instead when the data is severely corrupted. The theorem given in this correspondence is a strict solution of the problem, and it always gives the correct transformation parameters even when the data is corrupted.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;摘要——在计算机视觉的许多应用中，存在以下问题。给定两个点集合$\left\{a_i \right\} $和$\left\{y_i\right\}$； i = 1,2$\cdots$，n 在 m 维空间中，我们想找到使这两个点集合之间的最小均方误差最小的相似变换参数（旋转，平移和缩放）。最近，Aru等人和Horn等人提出了一个解决这个问题的解决方案。然而，当数据严重损坏时，他们的解决方案有时会失败，无法给出正确的旋转矩阵，反而给出一个反射矩阵。这篇文章给出的定理是该问题的一个严格解，无论数据是否损坏，它总是给出正确的变换参数。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Index Terms*—— Absolute orientation problem, computer vision, least-squares, motion estimation, singular value decomposition.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;索引术语——绝对定向问题，计算机视觉，最小二乘法，运动估计，奇异值分解。
### 引言
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In computer vision applications, we sometimes encounter the following mathematical problem. We are given two point patterns (sets of points) $\left\{x_i\right\}$ and $\left\{y_i\right\}$; i =  1,2 $\cdots$,n in m-dimensional space, and we want to find the similarity transformation parameters (R: rotation, t: translation, and c: scaling) giving the minimum value of the mean squared error $e^2(R, t,c)$ of these two point patterns.$$e^2(R,\boldsymbol{t},c)=\frac{1}{n}\sum_{i=1}^n\left\|\boldsymbol{y}_i-(cR\boldsymbol{x}_i+\boldsymbol{t})\right\|^2\tag{1}$$The dimensionality $m$ is usually 2 or 3.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;在计算机视觉应用中，我们有时会遇到以下数学问题：给定两个点模式（点集）$\left\{x_i\right\}$ 和 $\left\{y_i\right\}$，其中 $i = 1, 2, \cdots, n$，位于 m 维空间中。我们需要找到相似变换参数（R：旋转，t：平移，c：缩放），使得这两个点模式的均方误差 $e^2(R, t, c)$ 达到最小值。$$e^2(R,\boldsymbol{t},c)=\frac{1}{n}\sum_{i=1}^n\left\|\boldsymbol{y}_i-(cR\boldsymbol{x}_i+\boldsymbol{t})\right\|^2\tag{1}$$维度$m$通常是2或者3。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;This problem is sometimes called the absolute orientation problem ［1］, and an iterative algorithms for finding the solution ［2］ and a noniterative algorithm based on quaternions ［3］ are proposed for a 3-D problem. A good reference can be found in ［1］. Recently, Arun *et al.* ［4］ and Horn *et al.* ［1］ have presented a solution of this problem, which is based on the singular value decomposition of a covariance matrix of the data. Their solution, however, sometimes fails to give a correct rotation matrix and gives a reflection instead (det(R) = -1) when the data is severely corrupted.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;该问题有时被称为绝对定向问题，针对三维问题已经提出了迭代算法和基于四元数的非跌点算法来求解。关于这一问题，可以参考文献1。最近，Arun等人和Horn等人提出了一种基于数据协方差矩阵奇异值分解的解决方案。然而，当数据严重损坏时，他们的解决方案有时无法给出正确的旋转矩阵，而是给出一个反射矩阵（det(R) = -1）。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The theorem given in this correspondence is a strict solution of the problem, and it is derived by refining Arun's result. The theorem always gives the correct transformation parameters even when the data is corrupted.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;该文中给出的定理是通过对 Arun 的结果进行改进而得到的问题的严格解。即使数据遭到破坏，该定理也总是能够给出正确的变换参数。

### 变换参数的最小二乘估计
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In this section, we show a theorem which gives the least-squares estimation of similarity transformation parameters between two point patterns. Before showing the theorem, we prove a lemma, which gives the least-squares estimation of rotation parameters. This lemma is the main result of this correspondence.

在本节中，我们将展示一个定理，该定理给出了两个点模式之间相似变换参数的最小二乘估计。在展示定理之前，我们先证明一个引理，该引理给出了旋转参数的最小方差估计。这个引理是本文的主要结果。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Lemma: Let $A$ and $B$ be $m \times  n$ matrices, and $R$ an $m \times m$ rotation matrix,and $UDV^T$ a singular value decomposition of $AB^T$ $(UU^T =VV^T=I,D=diag(d_i),d_1\ge d_2\ge …\ge d_m \ge 0)$.
Then the minimum value of  ${\left \| A-RB \right \|^2}$ with respect to $R$ is $$\min_R\left\|A-RB\right\|^2=\left\|A\right\|^2+\left\|B\right\|^2-2\mathrm{tr}(DS)\tag{2}$$ where $$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if~}\mathrm{det}(AB^T)\geq0 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if~}\mathrm{det}(AB^T)<0.
\end{array}\right.\tag{3}$$ When $\mathrm{rank}(AB^T) \ge  m - 1$, the optimum rotation matrix $R$ which achieves the above minimum value is uniquely determined.$$R=USV^T\tag{4}$$
where $S$ in (4) must be chosen as $$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if}\quad\mathrm{det}(U)\mathrm{det}(V)=1 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if}\quad\mathrm{det}(U)\mathrm{det}(V)=-1
\end{array}\right.\tag{5}$$ when $\mathrm{det}(AB^T) = 0 (\mathrm{rank}(AB^T) = m - 1)$.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;设$A$和$B$为$m\times n$矩阵，$R$为$m\times m$的旋转矩阵，$UDV^T$为$AB^T$的奇异值分解$(UU^T=VV^T=I,D=diag(d_i),d_1\ge d_2\ge …\ge d_m \ge 0)$。那么对于$R$的最小值${\left \| A-RB \right \|^2}$，最小值是 $$\min_R\left\|A-RB\right\|^2=\left\|A\right\|^2+\left\|B\right\|^2-2\mathrm{tr}(DS)\tag{2}$$ 其中 $$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if~}\mathrm{det}(AB^T)\geq0 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if~}\mathrm{det}(AB^T)<0.
\end{array}\right.\tag{3}$$ 当$\mathrm{rank}(AB^T)\ge m-1$时，达到上述最小值的最优旋转矩阵$R$被唯一确定为$$R=USV^T\tag{4}$$,当$\mathrm{det}(AB^T)=0(\mathrm{rank}(AB^T)=m-1)$时，$S$ 在 (4) 中需要选择为 $$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if}\quad\mathrm{det}(U)\mathrm{det}(V)=1 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if}\quad\mathrm{det}(U)\mathrm{det}(V)=-1
\end{array}\right.\tag{5}$$。
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Proof of Lemma: Define an objective function $F$ as $$F=\|A-R B\|^{2}+\operatorname{tr}\left(L\left(R^{T} R-I\right)\right)+g\{\operatorname{det}(R)-1\}\tag{6}$$where $g$ is a Lagrange multiplier and $L$ is a symmetric matrix of Lagrange multipliers. The second and third term of $F$ represent the conditions for $R$ to be an orthogonal and proper rotation matrix respectively. Partial differentiations of $F$ with respect to $R$, $t$, and $c$ lead to the following system of equations [5]$$\frac{\partial F}{\partial R}=-2 A B^{T}+2 R B B^{T}+2 R L+g R=0\tag{7}$$ $${\frac{\partial F}{\partial L}=R^{T} R-I=0\tag{8}}$$ $${\frac{\partial F}{\partial g}=\operatorname{det}(R)-1=0\tag{9}}$$ where we used $$\frac{\partial}{\partial R}\mathrm{det}(R)=\mathrm{adj}\left(R^{T}\right)=\mathrm{det}\left(R^{T}\right)\left(R^{T}\right)^{-1}=R\tag{10}$$ since $R$ is a ratation matrix ($adj(R^T)$is an adjoint matrix of $R^T$) . From (7),$$RL^{\prime}=AB^{T},\quad\mathrm{where}L^{\prime}=BB^{T}+L+\frac{1}{2}gI.\tag{11}$$ By transposing the both sides of (11), we obtain the following equation (note that $L^\prime$ is symmetric). $$L^{^{\prime}}R^{T}=BA^{T}\tag{12}$$ If we multiply each side of (11) with each side of (12), respectively (13) is obtained since $R^TR= I$.$$L^{\prime2}=BA^TAB^T=VD^2V^T\tag{13}$$ Obviously $L^{\prime}$ and $L^{\prime2}$ are commutative ($L^{\prime}L^{\prime2} = L^{\prime2}L^{\prime}$),hence both can be reduced to diagonal forms by the same orthogonal matrix [6].
Thus we can write $$L^{\prime}= VDSV^T\tag{14}$$ where $S$ is $\mathrm{diag}(s_i)$, $s_i= 1$ or -1。<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Now ,from (14),$$\begin{aligned}
\operatorname*{det}(L^{\prime}) & =\mathrm{det}\left(VDSV^{T}\right) \\
 & =\mathrm{det}(V)\mathrm{det}(D)\mathrm{det}(S)\mathrm{det}\left(V^{T}\right) \\
 & =\mathrm{det}(D)\mathrm{det}(S).
\end{aligned}\tag{15}$$ On the other hand, from (11) $$\begin{aligned}
\operatorname*{det}(L^{\prime}) & =\mathrm{det}\left(R^{T}AB^{T}\right) \\
 & =\mathrm{det}\left(R^{T}\right)\mathrm{det}\left(AB^{T}\right) \\
 & =\mathrm{det}\left(AB^{T}\right).
\end{aligned}\tag{16}.$$ Thus,$$\mathrm{det}(D)\mathrm{det}(S)=\mathrm{det}\left(AB^T\right).\tag{17}$$ Since singular values are nonnegative, $\mathrm{det}(D) = d_1 d_2 \cdots d_m \ge0$.
Hence $\mathrm{det}(S)$ must be equal to 1 when $\mathrm{det}(AB^T)> 0$, and -1 when $\mathrm{det}(AB^T)< 0$.<br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Next, extremum values of $\left \| A-RB^2 \right \|$ is derived as follows: from (11) we have $$\begin{aligned}
\|A-RB\|^{2} & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(AB^{T}R^{T}\right) \\
 & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(R^{T}AB^{T}\right) \\
 & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(L^{\prime}\right).
\end{aligned}\tag{18}$$  Substituting (14) into (18), we have $$\begin{aligned}
\|A-RB\|^{2} & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(VDSV^{T}\right) \\
 & =\|A\|^{2}+\|B\|^{2}-2\mathrm{tr}(DS) \\
 & =\|A\|^{2}+\|B\|^{2}-2(d_{1}s_{1}+d_{2}s_{2}+\cdots+d_{m}s_{m}).
\end{aligned} \tag{19}$$ Thus, the minimum value of $\left \|  A-RB\right\|^2$ is obviously achieved when $s_1 = s_2 = \cdots s_{m-1}= 1$,$s_m=1$ if $\mathrm{det}(AB^T) \ge 0$, and $s_i =s_2 =\cdots s_{m-1} = 1$, $s_m = -1$ if $\mathrm{det}(AB^T) < 0$. This concludes the first half of the lemma.<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Next, we determine a rotation matrix $R$ achieving the above minimum value. When  $\mathrm{rank}(AB^T) = m$, $L^\prime$ is nonsingular, thus it has its inverse $L^{\prime-1}= (VDSV^T)^{-1} = VS^{-1}D^{-1}V^T = VD^{-1}SV^T$ (note that $S^{-1} = S$, $SD^{-1}= D^{-1}S$). Therefore, from (11) we have $$R=AB^TL^{\prime-1}=UDV^TVD^{-1}SV^T = USV^T\tag{20}$$ &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Finally,when $\mathrm{rank}(AB^T)=m-1$,from (11),(14) $$ RVDSV^T=UDV^T\tag{21}$$ Multiplying $V$ by both sides of (21) from the right and using $DS=D$ ( since $d_{m}=0$ and $s_{1}={s_{2}}=\cdots s_{m-1}=1$) $$RVD=UD \tag{22}$$
is obtained. If we define an orthogonal matrix $Q$ as follows:$$Q=U^{T}RV\tag{23}$$
we have $$QD=D\tag{24}$$.
Let the column vectors of  $Q$ be $\boldsymbol{q}_{1}$,$\boldsymbol{q}_{2}$,$\cdots$,$\boldsymbol{q}_{m}$ $\left(Q=[\boldsymbol{q}_{1},\boldsymbol{q}_{2},\cdots,\boldsymbol{q}_{m}]\right)$.The following equations are obtained by comparing both sides of (24).$$d_i\boldsymbol{q}_i=d_i\boldsymbol{e}_i\quad1\leq i\leq m-1 \tag{25}$$
Hence,$$\boldsymbol{q}_{i}=\boldsymbol{e}_{i}\quad1 \leq i \leq m-1 \tag{26}$$
where $\boldsymbol{e}_i$ is a unit vector which has 1 as an $i\text{th}$ element.$$\begin{matrix}
 \quad \quad \quad i\\
\boldsymbol{e}_i\:=\:\left(0,0,\cdots,1,\cdots,0\right)^T
\end{matrix}\tag{27}$$
The last column vector $\boldsymbol{q}_{m}$  of $Q$ is an orthogonal to all other vectors 
 $\boldsymbol{q}_i(1\leq i\leq m-1)$ since $Q$ is an orthogonal matrix. Thus we have $$\boldsymbol{q}_{m}=\boldsymbol{e}_{m}\quad\mathrm{or}\quad \boldsymbol{q}_{m}=-\boldsymbol{e}_{m}.\tag{28}$$
On the other hand,$$\begin{aligned}\mathrm{det}(Q)&=\mathrm{det}\Bigl(U^T\Bigr)\mathrm{det}(R)\mathrm{det}(V)\\&=\mathrm{det}(U)\mathrm{det}(V).\end{aligned}\tag{29}$$
Thus, $\mathrm{det}(Q)=1$ if $\mathrm{det}(U)\mathrm{det}(V)=1$  and $\mathrm{det}(Q)=-1$ if $\mathrm{det}(U)\mathrm{det}(V)=-1$. Therefore we have $$\begin{array}{c}R=UQV^T\\=USV^T\end{array}\tag{30}$$ where $$S=\left\{\begin{array}{ll}I&\quad\text{if}\:\mathrm{det}(U)\mathrm{det}(V)=1\\\text{diag}(1,1,\cdots,1,-1)&\quad\text{if}\:\mathrm{det}(U)\mathrm{det}(V)=-1.\end{array}\right.\tag{31}$$
$Q.E.D.$<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;We can derive the following theorem using this lemma.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**引理的证明：** 定义目标函数$F$如下：$$F=\|A-R B\|^{2}+\operatorname{tr}\left(L\left(R^{T} R-I\right)\right)+g\{\operatorname{det}(R)-1\}\tag{6}$$ 其中$g$是拉格朗日乘子，$L$是一个对称的拉格朗日乘子矩阵。$F$中的第二项和第三项分别代表了$R$作为正交矩阵和真旋转矩阵的条件。对$F$关于$R$，$t$，$c$求偏微分，得到如下方程组：$$\frac{\partial F}{\partial R}=-2 A B^{T}+2 R B B^{T}+2 R L+g R=0\tag{7}$$ $${\frac{\partial F}{\partial L}=R^{T} R-I=0\tag{8}}$$ $${\frac{\partial F}{\partial g}=\operatorname{det}(R)-1=0\tag{9}}$$,其中，由于$R$是旋转矩阵( $adj(R^T)$ 是$R^T$的伴随矩阵)，因此我们使用$$\frac{\partial}{\partial R}\mathrm{det}(R)=\mathrm{adj}\left(R^{T}\right)=\mathrm{det}\left(R^{T}\right)\left(R^{T}\right)^{- 1}=R\tag{10}$$。由( 7 )可得，$$RL^{\prime}=AB^{T},\quad\mathrm{其中}L^{\prime}=BB^{T}+L+\frac{1}{2}gI.\tag{11}$$由( 11 )的两边转置，我们得到如下方程(注意到$L^\prime$是对称的) . $$L^{^{\prime}}R^{T}=BA^{T}\tag{12}$$将( 11 )式两边同时乘以( 12 )式两边，由于$R^TR= I$，分别得到( 13 )式。$$L^{\prime2}=BA^TAB^T=VD^2V^T\tag{13}$$显然$L^{\prime}$和$L^{\prime2}$是可交换的( $L^{\prime}L^{\prime2} = L^{\prime2}L^{\prime}$ )，因此两者可以通过同一个正交矩阵化简为对角形式[ 6 ]。我们可以写出$$L^{\prime}= VDSV^T\tag{14}$$，其中$S$为$\mathrm{diag}(s_i)$，$s_i= 1$或-1。

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;现在由( 14 )式，可得$$\begin{aligned}
\operatorname*{det}(L^{\prime}) & =\mathrm{det}\left(VDSV^{T}\right) \\
 & =\mathrm{det}(V)\mathrm{det}(D)\mathrm{det}(S)\mathrm{det}\left(V^{T}\right) \\
 & =\mathrm{det}(D)\mathrm{det}(S)
\end{aligned}\tag{15}$$ 。
另外，由公式(11)可得
$$\begin{aligned}
\operatorname*{det}(L^{\prime}) & =\mathrm{det}\left(R^{T}AB^{T}\right) \\
 & =\mathrm{det}\left(R^{T}\right)\mathrm{det}\left(AB^{T}\right) \\
 & =\mathrm{det}\left(AB^{T}\right)
\end{aligned}\tag{16}$$ 
。
因此，$$\mathrm{det}(D)\mathrm{det}(S)=\mathrm{det}\left(AB^T\right)\tag{17}$$由于奇异值是非负的，$\mathrm{det}(D) = d_1 d_2 \cdots d_m \ge0$ 。
因此当$\mathrm{det}(AB^T)> 0$时，$\mathrm{det}(S)$必须等于1；当$\mathrm{det}(AB^T) < 0$时，$\mathrm{det}(S)$必须等于-1。

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;其次,我们得到 $\left \| A-RB^2 \right \|$ 的极值，如下：由公式(11)我们可得
$$\begin{aligned}
\|A-RB\|^{2} & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(AB^{T}R^{T}\right) \\
 & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(R^{T}AB^{T}\right) \\
 & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(L^{\prime}\right)
\end{aligned}\tag{18}$$
 。将(14)代入, 我们可得 $$\begin{aligned}
\|A-RB\|^{2} & =\left\|A\right\|^{2}+\left\|B\right\|^{2}-2\mathrm{tr}\left(VDSV^{T}\right) \\
 & =\|A\|^{2}+\|B\|^{2}-2\mathrm{tr}(DS) \\
 & =\|A\|^{2}+\|B\|^{2}-2(d_{1}s_{1}+d_{2}s_{2}+\cdots+d_{m}s_{m}).
\end{aligned} \tag{19}$$ 因此，当$s_1 = s_2 = \cdots s_{m- 1}= 1$，$s_m=1$ 若$\mathrm{det}(AB^T) \ge 0$，当$s_i =s_2 =\cdots s_{m-1} = 1$，$s_m = -1$若$\mathrm{det}(AB^T) < 0$，$\left \|  A- RB\right\|^2$可明显达到最小值。这就推断出引理的前半部分。

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;接下来，我们确定一个达到上述最小值的旋转矩阵$R$。当$\mathrm{rank}(AB^T) = m$时，$L^\prime$是非奇异的，因此它有它的逆$L^{\prime- 1}= (VDSV^T)^{- 1} = VS^{- 1}D^{- 1}V^T = VD^{- 1}SV^T$(注意到$S^{- 1} = S$、$SD^{- 1}= D^{- 1}S$)。因此，由(11)式可得$$R=AB^TL^{\prime- 1}=UDV^TVD^{- 1}SV^T = USV^T\tag{20}$$

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;最后,当 $\mathrm{rank}(AB^T)=m-1$时,从公式 (11),(14) 可得$$ RVDSV^T=UDV^T\tag{21}$$ 
公式(21)两侧右边都乘以$V$以及代入$DS=D$ ( 由于 $d_{m}=0$ 和 $s_{1}={s_{2}}=\cdots s_{m-1}=1$)可得$$RVD=UD \tag{22}$$ 。如果我们定义一个正交矩阵$Q$如下: $$Q=U^{T}RV\tag{23}$$
我们可得 $$QD=D\tag{24}$$。
设$Q$的列向量分别为$\boldsymbol{q}_{1}$,$\boldsymbol{q}_{2}$,$\cdots$,$\boldsymbol{q}_{m}$，$\left(Q=[\boldsymbol{q}_{1},\boldsymbol{q}_{2},\cdots,\boldsymbol{q}_{m}]\right)$。通过比较( 24 )式的两边得到下面的方程。$$d_i\boldsymbol{q}_i=d_i\boldsymbol{e}_i\quad1\leq i\leq m-1 \tag{25}$$
因此,$$\boldsymbol{q}_{i}=\boldsymbol{e}_{i}\quad1 \leq i \leq m-1 \tag{26}$$
其中 $\boldsymbol{e}_i$ 是一个第$i$项元素为1的单位向量。$$\begin{matrix}
 \quad \quad \quad i\\
\boldsymbol{e}_i\:=\:\left(0,0,\cdots,1,\cdots,0\right)^T
\end{matrix}\tag{27}$$
因为$Q$是正交矩阵，$Q$的最后一列向量$\boldsymbol{q}_m$是与其它向量$q_i(1\leq i\leq m-1)$ 正交的。因此我们可得 $$\boldsymbol{q}_{m}=\boldsymbol{e}_{m}\quad\mathrm{或者}\quad \boldsymbol{q}_{m}=-\boldsymbol{e}_{m}\tag{28}$$ 。
另外,$$\begin{aligned}\mathrm{det}(Q)&=\mathrm{det}\Bigl(U^T\Bigr)\mathrm{det}(R)\mathrm{det}(V)\\&=\mathrm{det}(U)\mathrm{det}(V).\end{aligned}\tag{29}$$
因此,如果 $\mathrm{det}(U)\mathrm{det}(V)=1$则 $\mathrm{det}(Q)=1$ ，如果$\mathrm{det}(U)\mathrm{det}(V)=-1$ 则$\mathrm{det}(Q)=-1$ 。因此，$$\begin{array}{c}R=UQV^T\\=USV^T\end{array}\tag{30}$$ 其中 $$S=\left\{\begin{array}{ll}I&\quad\text{if}\:\mathrm{det}(U)\mathrm{det}(V)=1\\\text{diag}(1,1,\cdots,1,-1)&\quad\text{if}\:\mathrm{det}(U)\mathrm{det}(V)=-1.\end{array}\right.\tag{31}$$
$Q.E.D.$<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;利用这个引理，我们可以得到下面的定理。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Theorem*: Let $X =\left\{x_1, x_2,\cdots,x_n \right\} $ and $Y = \left\{y_1,y_2,\cdots,y_n\right\}$ be corresponding point patterns in $m$-dimensional space. The minimum value $\varepsilon ^2$ of the mean squared error $$e^2(R,t,c)=\frac{1}{n}\sum_{i=1}^n\left\|y_i-(cR{x}_i+{t})\right\|^2 \tag{32}$$ of these two point patterns with respect to the similarity transformation parameters ($R$: rotation, $t$: translation, and $c$: scaling) is given as follows :