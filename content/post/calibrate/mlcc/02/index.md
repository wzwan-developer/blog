---
title: "Chapter 02: LiDAR-Camera Extrinsic Calibration"
description: 《 Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization》解读系列。
date: 2024-10-09T17:22:01+08:00
image: camera_voxel.png
math: true
slug: mlcc/02
license: 
hidden: false
comments: true
draft: true
categories:
- Calibrate
tags:
- mlcc
---
## 文章内容
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;With the LiDAR extrinsic parameter $\mathcal{E}_L$  and pose trajectory $\mathcal{S}$ computed above, we obtain a dense global point cloud by transforming all LiDAR points to the base LiDAR frame. Then, the extrinsic $\mathcal{E}_C$ is optimized by minimizing the summed distance between the back-projected LiDAR edge feature points and the image edge feature points. Two types of LiDAR edge points could be extracted from the point cloud. One is the depth-discontinuous edge between the foreground and background objects, and the other is the depth-continuous edge between two neighboring non-parallel planes. As explained in our previous work [28], depth-discontinuous edges suffer from foreground inflation and bleeding points phenomenon; we hence use depth-continuous edges to match the point cloud and images.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;利用上述计算得到的激光雷达外参参数 $\mathcal{E}_L$ 和位姿轨迹 $\mathcal{S}$，我们将所有激光雷达点转换到基激光雷达坐标系中，从而获得一个密集的全局点云。接着，通过最小化反向投影的激光雷达边缘特征点与图像边缘特征点之间的累积距离来优化相机外参 $\mathcal{E}_C$。可以从点云中提取两种类型的激光雷达边缘点。一种是前景和背景物体之间的深度不连续边缘；另一种是非平行邻近平面之间的深度连续边缘。正如我们先前的工作 [28] 中所解释的那样，深度不连续边缘容易出现前景膨胀和漂移点现象；因此，我们使用深度连续边缘来进行点云与图像的匹配。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In [28], the LiDAR point cloud is segmented into voxels with uniform sizes, and the planes inside each voxel are estimated by the **RANSAC** algorithm. In contrast, our method uses the same adaptive voxel map obtained in Sec. III-B. We calculate the angle between their containing plane normals for every two adjacent voxels. If this angle exceeds a threshold, the intersection line of these two planes is extracted as the depthcontinuous edge, as shown in Fig. 5. We choose to implement the Canny algorithm for image edge features to detect and extract.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;在文献[28]中，LiDAR点云被分割成大小均匀的体素，然后通过**RANSAC**算法估计每个体素内的平面。相比之下，我们的方法使用了在第III-B节中获得的自适应体素地图。我们计算每两个相邻体素中包含平面法线之间的夹角。如果该夹角超过某一阈值，则提取这两个平面的交线作为深度连续边缘，如图5所示。我们选择使用Canny算法来检测并提取图像中的边缘特征。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Suppose ${}^{G}p_{i}$ represents the $i$-th point from a LiDAR edge feature extracted above in global frame. With pin-hole camera and its distortion model,${}^{G}p_{i}$ projected onto the image taken by camera $C_l$ at $t_j$, i.e., $I_{i,j}$ 
by $${}^{\mathbf{I}_{l,j}}\mathbf{p}_{i}=\mathbf{f}\left(\boldsymbol{\pi}\left({{}^{C_l}_{L_{0}}}\mathbf{T}\left({}^{G}_{L_0}\mathbf{T}_{t_{j}}\right)^{-1}{}^{G}\mathbf{p}_{i}\\\right)\right)\tag{7}$$
where f(·) is the camera distortion model and π(·) is the projection model. Let $\mathcal{I}_i$ represent the set of images that capture the point ${}^{G}p_{i}$, i.e., $\mathcal{I}_i = {I_{l,j}}$. For each ${}^{I_{l,j}}p_i$, the $\kappa $ nearest image edge feature points  $q_k$ on $I_{l,j}$ are searched. The normal vector $n_{i,l,j}$ of the edge formed by these $\kappa $ points is thus the eigenvector corresponding to the minimum eigenvalue of $A_{i,l,j}$ that $$ \mathbf{A}_{i,l,j}=\sum_{k=1}^\kappa(\mathbf{q}_k-\mathbf{q}_{i,l,j})(\mathbf{q}_k-\mathbf{q}_{i,l,j})^T,\mathbf{q}_{i,l,j}=\frac1\kappa\sum_{k=1}^\kappa\mathbf{q}_k\tag{8}$$.
The residual originated from this LiDAR camera correspondence is defined as $$\mathbf{r}_{i,l,j}=\mathbf{n}_{i,l,j}^{T}\left({}^{\mathbf{I}_{l,j}}\mathbf{p}_{i}-\mathbf{q}_{i,l,j}\right).\tag{9}$$
Collecting all such correspondences, the extrinsic ECcalibration problem could be formulated as $$\mathcal{E}_{C}^{*}=\arg\min_{\mathcal{E}_{C}}\sum_{i}\sum_{\mathbf{I}_{l,j}\in\mathcal{I}_{i}}\left(\mathbf{n}_{i,l,j}^{T}\left({}^{\mathbf{I}_{l,j}}\mathbf{p}_{i}-\mathbf{q}_{i,l,j}\right)\right)\tag{10}$$.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;假设${}^{G}p_{i}$是从全局坐标系中提取的第i个激光雷达边缘特征点。利用针孔相机及其畸变模型，将${}^{G}p_{i}$投影到 $C_l$在时刻$t_j$时刻拍摄的图像$I_{i,j}$上，数学表达如下:
$$
{}^{\mathbf{I}_{l,j}}\mathbf{p}_{i}=\mathbf{f}\left(\boldsymbol{\pi}\left({{}^{C_l}_{L_{0}}}\mathbf{T}\left({}^{G}_{L_0}\mathbf{T}_{t_{j}}\right)^{-1}{}^{G}\mathbf{p}_{i}\\\right)\right)\tag{7}
$$。
其中$\mathbf{f}(\cdot)$是相机畸变模型，$\boldsymbol{\pi}(\cdot)$是投影模型.令$\mathcal{I}_i$表示捕捉到了点${}^{G}p_{i}$的图像集合，即$\mathcal{I}_i = {I_{l,j}}$。对于每个 ${}^{I_{l,j}}p_i$，在图像上搜索其$\kappa $个最近邻的图像边缘特征点$q_k$.由于这$\kappa$个点形成的边缘的法向量$n_{i,l,j}$即为矩阵$A_{i,l,j}$的最小特征值对应的特征向量，其中$$ \mathbf{A}_{i,l,j}=\sum_{k=1}^\kappa(\mathbf{q}_k-\mathbf{q}_{i,l,j})(\mathbf{q}_k-\mathbf{q}_{i,l,j})^T,\mathbf{q}_{i,l,j}=\frac1\kappa\sum_{k=1}^\kappa\mathbf{q}_k\tag{8}$$.
由此对应激光雷达与相机对应关系产生的残差定义为：$$\mathbf{r}_{i,l,j}=\mathbf{n}_{i,l,j}^{T}\left({}^{\mathbf{I}_{l,j}}\mathbf{p}_{i}-\mathbf{q}_{i,l,j}\right).\tag{9}$$
收集所有的这样的对应关系，相机外参校准问题可以表述为$$\mathcal{E}_{C}^{*}=\arg\min_{\mathcal{E}_{C}}\sum_{i}\sum_{\mathbf{I}_{l,j}\in\mathcal{I}_{i}}\left(\mathbf{n}_{i,l,j}^{T}\left({}^{\mathbf{I}_{l,j}}\mathbf{p}_{i}-\mathbf{q}_{i,l,j}\right)\right)\tag{10}$$.

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Inspecting the residual in (9), we find the ${}^{I_{l,j}}p_i$ is dependent on LiDAR poses ${}^{G}_{L_0}T_{t_j}$. This is due to the reason that LiDARs may have FoV overlap with cameras at different times (as in Fig. 2). Since ${}^{G}_{L_0}T_{t_j}\in\mathcal{S}$ has been well estimated from Sec. III-C, we keep them fixed in this step. Moreover, the $n_{i,l,j}$ and $q_{i,l,j}$ are also implicitly dependent on $\mathcal{E}_C$, since both ni,l,jand qi,l,jare related with nearest neighbor search.
The complete derivative of (10) to the variable $\mathcal{E}_C$ would be too complicated. In this paper, to simplify the optimization problem, we ignore the influence of camera extrinsic on $n_{i,l,j}$ and $q_{i,l,j}$. This strategy works well in practice as detailed in Sec. IV-B.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;检查公式 (9) 中的残差，我们发现 ${}^{I_{l,j}}p_i$ 依赖于激光雷达的姿态 ${}^{G}_{L_0}T_{t_j}$。这是由于激光雷达可能在不同时间与相机存在视场重叠（如图2所示）。因为 ${}^{G}_{L_0}T_{t_j}\in\mathcal{S}$ 已经根据第三节C部分很好地估计出来了，在此步骤中我们将它们固定不变。此外，$n_{i,l,j}$ 和 $q_{i,l,j}$ 也隐式地依赖于外参 $\mathcal{E}_C$，因为两者都与最近邻搜索相关。
对变量 $\mathcal{E}_C$ 完整求导将会过于复杂。在本文中，为了简化优化问题，我们忽略了相机外参对 $n_{i,l,j}$ 和 $q_{i,l,j}$ 的影响。如第四节B部分详细说明的那样，这一策略在实践中表现良好。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The non-linear optimization (10) is solved with **LM** method by approximating the residuals with their first order derivatives (11). The optimal $\mathcal{E}^{\ast}_{C}$ is then obtained by iteratively solving (11) and updating $\delta{x}$ to $x$ using the $\boxplus$ operation [31].$$\delta\mathbf{x}=-\left(\mathbf{J}^{T}\mathbf{J}+\mu\mathbf{I}\right)^{-1}\mathbf{J}^{T}\mathbf{r},\tag{11}$$ where $$
\begin{align*}
\delta{x}=\begin{bmatrix}\cdots\quad{}^{C_1}_{L_0}\phi^{T}\quad\delta{{}^{C_1}_{L_0}t^{T}}\quad\cdots\end{bmatrix}\in\mathbb{R}^{6h}\\
x=\begin{bmatrix}\cdots\quad{}^{C_1}_{L_0}R\quad{}^{C_1}_{L_0}t\quad\cdots\end{bmatrix}\\
J=\begin{bmatrix}\cdots\quad J^{T}_{p}\quad\cdots\end{bmatrix}^{T},r=\begin{bmatrix}\cdots\quad{r_p}\quad\cdots\end{bmatrix}^{T},
\end{align*} $$
with $J_p$ and $r_p$ begin the sum of $J_{i,l,j}$ and $r_{i,l,j}$ when $l = p$: $$ \begin{align*}
&\mathbf{J}_{i,l,j} =\mathbf{n}_{i,l,j}^T\frac{\partial\mathbf{f}(\mathbf{p})}{\partial\mathbf{p}}\frac{\partial\boldsymbol{\pi}(\mathbf{P})}{\partial\mathbf{P}}\left[-_{L_0}^{C_l}\mathbf{R}\left(^{L_0}\mathbf{p}_i\right)^{\wedge}\quad\mathbf{I}\right]\in\mathbb{R}^{1\times6} \\
&^{L_0}\mathbf{p}_i =\left({}_{L_0}^G\mathbf{T}_{t_j}\right)^{-1}{}^G\mathbf{p}_i. 
\end{align*}\tag{12} $$

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;非线性优化问题 (10) 通过使用 **LM**（Levenberg-Marquardt）方法并用一阶导数 (11) 近似残差来求解。通过迭代求解 (11) 并使用 $\boxplus$ 操作更新 $\delta{x}$ 到 $x$ 来获得最优的 $\mathcal{E}^{\ast}_{C}$：$$\delta\mathbf{x}=-\left(\mathbf{J}^{T}\mathbf{J}+\mu\mathbf{I}\right)^{-1}\mathbf{J}^{T}\mathbf{r},\tag{11}$$
其中，
$$
\begin{align*}
\delta{x}=\begin{bmatrix}\cdots\quad{}^{C_1}_{L_0}\phi^{T}\quad\delta{{}^{C_1}_{L_0}t^{T}}\quad\cdots\end{bmatrix}\in\mathbb{R}^{6h}\\
x=\begin{bmatrix}\cdots\quad{}^{C_1}_{L_0}R\quad{}^{C_1}_{L_0}t\quad\cdots\end{bmatrix}\\
J=\begin{bmatrix}\cdots\quad J^{T}_{p}\quad\cdots\end{bmatrix}^{T},r=\begin{bmatrix}\cdots\quad{r_p}\quad\cdots\end{bmatrix}^{T},
\end{align*} 
$$
并且当 $l = p$ 时，$J_p$ 和 $r_p$ 分别为 $J_{i,l,j}$ 和 $r_{i,l,j}$ 的和：
$$ \begin{align*}
&\mathbf{J}_{i,l,j} =\mathbf{n}_{i,l,j}^T\frac{\partial\mathbf{f}(\mathbf{p})}{\partial\mathbf{p}}\frac{\partial\boldsymbol{\pi}(\mathbf{P})}{\partial\mathbf{P}}\left[-_{L_0}^{C_l}\mathbf{R}\left(^{L_0}\mathbf{p}_i\right)^{\wedge}\quad\mathbf{I}\right]\in\mathbb{R}^{1\times6} \\
&^{L_0}\mathbf{p}_i =\left({}_{L_0}^G\mathbf{T}_{t_j}\right)^{-1}{}^G\mathbf{p}_i. 
\end{align*}\tag{12} 
$$

## 理论推导