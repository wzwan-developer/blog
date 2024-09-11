---
title: "Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization"
description: 多雷达外参标定章节
date: 2024-09-11T09:05:20+08:00
image: lidar_voxel.png
math: true
license: 
hidden: false
comments: true
categories:
- Calibrate
tags:
- 标定论文
---
## 文章内容

> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;With adaptive voxelization, we can obtain a set of voxels of different sizes. Each voxel contains points that are roughly on a plane and creates a planar constraint for all LiDAR poses that  have points in this voxel. More specifically, considering the $l-th$ voxel consisting of a group of points $\mathcal{P}_{l}={_{}^{G}P_{L_{i},t_{j}}}$ scanned by $L_{i} \in \mathcal{L}$ at times $t_{j} \in \mathcal{T}$. We define a point cloud consistency  indicator $_{c_{l}}(_{L_{i}}^{G}T_{t_{j}})$ which forms a factor on $\mathcal{S}$ and $\mathcal{E}_{L}$ shown in Fig. 4(a). Then, the base LiDAR trajectory and extrinsic are estimated by optimizing the factor graph. A natural choice   for the consistency indicator $c_{l}(\cdot)$ would be the summed Euclidean distance between each $_{}^{G}P_{L_{i},t_{j}}$ the plane to be estimated (see Fig. 4(b)). Taking account of all such indicators within the voxel map, we could formulate the problem as $$\arg\min_{{\mathcal{S},\mathcal{E}_{L},\mathbf{n}_{l},\mathbf{q}_{l}}}\sum_{l}\underbrace{{\left(\frac{1}{N_{l}}\sum_{k=1}^{{N_{l}}}\left(\mathbf{n}_{l}^{T}\left(^{G}\mathbf{p}_{k}-\mathbf{q}_{l}\right)\right)^{2}\right)}}_{{l\mathrm{-th~factor}}}$$， where $_{}^{G}p_{k}\in \mathcal{P}_{l}$, $N_{l}$ is the total number of points in $\mathcal{P}_{l}$, $n_{l}$ is the normal vector of the plane and $q_{l}$ is a point on this plane.
![Fig.4](graph1.png)
Fig.4 :(a) The $l-th$ factor item relating to $\mathcal{S}$ and $\mathcal{E}_{L}$ with $L_{i} \in \mathcal{L}$ and $t_{j} \in \mathcal{T}$ . (b) The distance from the point $_{}^{G}p_{k}$ to the plane $\pi$.

 &nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;通过自适应体素化，我们可以获得一组不同大小的体素。每个体素包含大致在一个平面上的点，并为所有包含在此体素内的雷达姿态创建一个平面约束。更具体地说，考虑由$L_{i} \in \mathcal{L}$在时刻$t_{j} \in \mathcal{T}$扫描的一组点组成的第$l$个体素。我们定义了一个点云一致性指标$_{c_{l}}(_{L_{i}}^{G}T_{t_{j}})$ ，它在图4(a)中形成了$\mathcal{S}$ 和 $\mathcal{E}_{L}$上的因子。然后，通过优化因子图来估计基准雷达的轨迹和外参。对于一致性指标$c_{l}(\cdot)$的一个自然选择是计算每个$_{}^{G}P_{L_{i},t_{j}}$到平面的欧几里得距离之和（见图4(b)）。考虑到体素图中所有这样的指标，我们可以将问题表述为$$\arg\min_{{\mathcal{S},\mathcal{E}_{L},\mathbf{n}_{l},\mathbf{q}_{l}}}\sum_{l}\underbrace{{\left(\frac{1}{N_{l}}\sum_{k=1}^{{N_{l}}}\left(\mathbf{n}_{l}^{T}\left(^{G}\mathbf{p}_{k}-\mathbf{q}_{l}\right)\right)^{2}\right)}}_{{l\mathrm{-th~factor}}}$$，其中 $_{}^{G}p_{k}\in \mathcal{P}_{l}$，$N_{l}$ 是 $\mathcal{P}_{l}$中所有点的总点数, $n_{l}$ 是平面的法向量， $q_{l}$ 是平面中的一点。
 ![图4](graph1.png)
 Fig.4 :(a) 第$l$ 个因子项，涉及$\mathcal{S}$ 和 $\mathcal{E}_{L}$，其中  $L_{i} \in \mathcal{L}$ 且 $t_{j} \in \mathcal{T}$ 。 (b)点 $_{}^{G}p_{k}$到平面$\pi$的距离.

 > &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;It is noticed that the optimization variables $(n_{l}, q_{l})$ in (2) could be analytically solved (see Appendix A) and the resultant cost function (3) is over the LiDAR pose $_{L_{i}}^{G}T_{t_{j}}$ (hence the base LiDAR trajectory $\mathcal{S}$ and extrinsic $\mathcal{E}_{L}$) only, as follows $$ \arg\min_{\mathcal{S},\mathcal{E}_{L}}\sum_{l}^{}\lambda_{3}(A_{l}) $$ where $\lambda_{3}(A_{l})$ denotes the minimal eigenvalue of matrix $A_{l}$ defined as $$ A_{l}=\frac{1}{N_{l}}\sum_{k=1}^{N_{l}}{_{}^{G}p_{k} \cdot_{}^{G}p_{k}^{T}-q_{l}^{\ast}\cdot {q_{l}^{\ast}}^{T}},q_{l}^{\ast} =\frac{1}{N_{l}}\sum_{k=1}^{N_{l}}{_{}^{G}p_{k}}$$ . To allow efficient optimization in (3), we derive the closedform derivatives w.r.t the optimization variable $x$ up to secondorder (the detailed derivation from (3) to (5) is elaborated in Appendix B):$$\lambda_3(\mathbf{x}\boxplus\delta\mathbf{x})\approx\lambda_3(\mathbf{x})+\mathbf{\bar{J}}\delta\mathbf{x}+\frac12\delta\mathbf{x}^T\mathbf{\bar{H}}\delta\mathbf{x}$$ ,where $\bar{J}$ is the Jacobian matrix, and $\bar{H}$  is the Hessian matrix.The $\delta{x}$  is a small perturbation of the optimization variable $x$:$$\mathbf{x}=[\underbrace{\cdots_{L_{0}}^{G}\mathbf{R}_{t_{j}}\quad L_{0}^{G}\mathbf{t}_{t_{j}}\cdots}_{\mathcal{S}}\underbrace{\cdots_{L_{i}}^{L_{0}}\mathbf{R}_{L_{i}}^{L_{0}}\mathbf{t}\cdots}_{\mathcal{E}_{L}}]$$ .Then the optimal $x^{\ast}$ could be determined by iteratively solving (6) with the LM method and updating the $\delta{x}$ to $x$.
$$(\bar{\mathbf{H}}+\mu\mathbf{I}) \delta\mathbf{x}=-\bar{\mathbf{J}}^T$$

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;注意到优化变量$(n_{l}, q_{l})$在方程(2)中可以解析求解（详见附录A）,由此得到的损失函数(3)仅关于雷达姿态$_{L_{i}}^{G}T_{t_{j}}$(及基准雷达轨迹$\mathcal{S}$和外参$\mathcal{E}_{L}$),如下所示：
$$ \arg\min_{\mathcal{S},\mathcal{E}_{L}}\sum_{l}^{}\lambda_{3}(A_{l}) $$
其中$\lambda_{3}(A_{l})$表示矩阵 $A_{l}$的最小特征值， $A_{l}$定义为$$ A_{l}=\frac{1}{N_{l}}\sum_{k=1}^{N_{l}}{_{}^{G}p_{k} \cdot_{}^{G}p_{k}^{T}-q_{l}^{\ast}\cdot {q_{l}^{\ast}}^{T}},q_{l}^{\ast} =\frac{1}{N_{l}}\sum_{k=1}^{N_{l}}{_{}^{G}p_{k}}$$
。为了使式(3)中的优化高效，我们推导了优化变量$x$的二阶闭式导数(从(3)到(5)的详细推导见附录B)：
$$\lambda_3(\mathbf{x}\boxplus\delta\mathbf{x})\approx\lambda_3(\mathbf{x})+\mathbf{\bar{J}}\delta\mathbf{x}+\frac12\delta\mathbf{x}^T\mathbf{\bar{H}}\delta\mathbf{x}$$
。其中$\bar{J}$是雅可比矩阵，$\bar{H}$是海森矩阵。$\delta{x}$是优化变量$x$的小扰动：
$$\mathbf{x}=[\underbrace{\cdots_{L_{0}}^{G}\mathbf{R}_{t_{j}}\quad L_{0}^{G}\mathbf{t}_{t_{j}}\cdots}_{\mathcal{S}}\underbrace{\cdots_{L_{i}}^{L_{0}}\mathbf{R}_{L_{i}}^{L_{0}}\mathbf{t}\cdots}_{\mathcal{E}_{L}}]$$
。然后，最优解$x^{\ast}$可以通过迭代求解公式(6)并使用LM的方法更新$\delta{x}$到$x$来确定。
$$(\bar{\mathbf{H}}+\mu\mathbf{I}) \delta\mathbf{x}=-{\bar{\mathbf{J}}}^T$$
## 理论推导




## 代码详解
## 参考文献
> [1]*[《Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization》](https://arxiv.org/pdf/2109.06550)*
