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
{ll}I & \quad\mathrm{if~}\det(AB^T)\geq0 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if~}\det(AB^T)<0.
\end{array}\right.\tag{3}$$ When $rank(AB^T) \ge  m - 1$, the optimum rotation matrix $R$ which achieves the above minimum value is uniquely determined.$$R=USV^T\tag{4}$$
where $S$ in (4) must be chosen as$$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if}\quad\det(U)\det(V)=1 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if}\quad\det(U)\det(V)=-1
\end{array}\right.\tag{5}$$when $det (AB^T) = 0 (rank(AB^T) = m - 1)$.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;设$A$和$B$为$m\times n$矩阵，$R$为$m\times m$的旋转矩阵，$UDV^T$为$AB^T$的奇异值分解$(UU^T=VV^T=I,D=diag(d_i),d_1\ge d_2\ge …\ge d_m \ge 0)$。那么对于$R$的最小值${\left \| A-RB \right \|^2}$，最小值是 $$\min_R\left\|A-RB\right\|^2=\left\|A\right\|^2+\left\|B\right\|^2-2\mathrm{tr}(DS)\tag{2}$$ 其中 $$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if~}\det(AB^T)\geq0 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if~}\det(AB^T)<0.
\end{array}\right.\tag{3}$$ 当$rank(AB^T)\ge m-1$时，达到上述最小值的最优旋转矩阵$R$被唯一确定为$$R=USV^T\tag{4}$$,当$det(AB^T)=0(rank(AB^T)=m-1)$时，$S$ 在 (4) 中需要选择为 $$S=\left\{
\begin{array}
{ll}I & \quad\mathrm{if}\quad\det(U)\det(V)=1 \\
\operatorname{diag}(1,1,\cdots,1,-1) & \quad\mathrm{if}\quad\det(U)\det(V)=-1
\end{array}\right.\tag{5}$$。