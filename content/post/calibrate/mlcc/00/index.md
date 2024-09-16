---
title: "Chapter 00: Adaptive Voxelization"
description: 《 Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization》解读系列。
date: 2024-09-16T15:55:17+08:00
image: lidar_voxel.png
math: true
slug: mlcc/00
license: 
hidden: false
comments: true
categories:
- Calibrate
tags:
- mlcc
---
## 文章内容
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;To find the correspondences among different LiDAR scans, we assume the initial base LiDAR trajectory $\mathcal{S}$, LiDAR extrinsic $\mathcal{E}_L$, and camera extrinsic $\mathcal{E}_C$ are available. The initial base LiDAR trajectory $\mathcal{S}$ could be obtained by an online LiDAR SLAM (e.g., [3]), and the initial extrinsic could be obtained from the CAD design or a rough Hand-Eye calibration [14].
Our previous work [5] extracts edge and plane feature points from each LiDAR scan and matches them to the nearby edge and plane points in the map by a $k$-nearest neighbor search ($k-NN$). This would repeatedly build a $k$-d tree of the global map at each iteration. In this paper, we use a more efficient voxel map proposed in [4] to create correspondences among all LiDAR scans.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;为了找到不同雷达扫描之间的对应关系，我们假设初始的基准雷达轨迹$\mathcal{S}$、雷达外参$\mathcal{E}_L$、相机外参$\mathcal{E}_C$是可用的。初始的基准雷达轨迹$\mathcal{S}$可以通过实时雷达SLAM获得，而初始的外参可以通过CAD设计或者从粗略的手眼标定中获得。我们的前期工作，从每个雷达扫描中提取边缘和平面特征，并通过最邻域搜索，将他们匹配到地图中的临近边缘和平面。这会在每次迭代中重复构建全局地图的$k$-d树，在本文中，我们使用在文献4中提出的一种更为高效的体素地图来创建所有雷达扫描之间的对应关系。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The voxel map is built by cutting the point cloud (registered using the current $\mathcal{S}$ and $\mathcal{E}_L$) into small voxels such that all points in a voxel roughly lie on a plane (with some adjustable tolerance). The main problem of the fixed-resolution voxel map is that if the resolution is high, the segmentation would be too time-consuming, while if the resolution is too low, multiple small planes in the environments falling into the same voxel would not be segmented. To best adapt to the environment, we implement an adaptive voxelization process. More specifically, the entire map is first cut into voxels with a pre-set size (usually large, e.g., 4m). Then for each voxel, if the contained points from all LiDAR scans roughly form a plane (by checking the ratio between eigenvalues), it is treated as a planar voxel; otherwise, they will be divided into eight octants, where each will be examined again until the contained points roughly form a plane or the voxel size reaches the pre-set minimum lower bound. Moreover, the adaptive voxelization is performed directly on the LiDAR raw points, so no prior feature points extraction is needed as in [5].

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;体素地图通过将点云（使用当前的 $\mathcal{S}$和 $\mathcal{E}_L$ 进行配准）切割成小的体素来构建，使得体素内的所有点大致位于同一平面上（具有一定的可调容差）。固定分辨率体素图的主要问题是，如果分辨率太高，分割将会非常耗时；而如果分辨率太低，环境中多个小平面落在同一个体素内时则无法进行分割。为了更好地适应环境，我们实现了自适应体素化过程。
具体来说，整个地图首先被切割成预设大小的体素（通常较大，例如4米）。然后对于每个体素，如果所有LiDAR扫描中包含的点大致形成一个平面（通过检查特征值之间的比率来判断），则将其视为平面体素；否则，这些体素将被分成八个八分之一体素（octants），每个都将再次进行检查，直到包含的点大致形成一个平面，或者体素尺寸达到预设的最小下限。此外，自适应体素化直接在LiDAR的原始点上执行，因此不需要像文献[5]那样预先提取特征点。

>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Fig. 3 shows a typical result of the adaptive voxelization process in a complicated campus environment. As can be seen, this process is able to segment planes of different sizes, including large planes on the ground, medium planes on the building walls, and tiny planes on tree crowns.
![图3](adaptive_voxelization.png)
Fig. 3: A) LiDAR point cloud segmented with the adaptive voxelization. Points within the same voxel are colored identically. The detailed adaptive voxelization of points in the dashed white rectangle could be viewed in B) colored points and C) original points. The default size for the initial voxelization is 4m, and the minimum voxel size is 0.25m.


图3展示了一个复杂校园环境中自适应体素化过程的典型结果。如图所示，该过程能够分割出不同大小的平面，包括地面上的大平面、建筑物墙面上的中等平面以及树冠上的小平面。
![图3](adaptive_voxelization.png)
图3：(A) 使用自适应体素化分割的LiDAR点云。同一体素内的点被赋予相同的颜色。白色虚线矩形区域内的详细自适应体素化效果可以在 (B) 彩色点云和 (C) 原始点云中查看。初始体素化的默认大小为4米，而最小的体素大小为0.25米。

## 相关理论
论文中关于自适应体素并未提及到重要信息，但是提到了参考文献4[《BALM: Bundle Adjustment for Lidar Mapping》](https://www.arxiv.org/pdf/2010.08215)。
### 自适应体素化过程
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;我们在默认大小的3D空间中重复体素化，如果当前体素中的所有特征点都位于平面，则将当前体素与包含的特征点一起保存在内存中；否则，将当前体素分解为八个八分体，并继续检查每个八分体直到达到最小尺寸。在具体实现过程中有以下细节：
- 如果一个体素内包含太多的特征点，则会导致文章[《Chapter 01: Multi-LiDAR Extrinsic Calibration》]({{< ref "/post/calibrate/mlcc/01/index.md" >}})中推导过程章节中二阶闭式导数中的Hessian矩阵维度过高，在这种情况下，我们可以将点进行平均，以实现降采样但不降低映射一致性；
- 同时，二阶闭式导数中的Hessian矩阵推导过程中提到$\lambda_m\ne\lambda_n$,因此当遇到$\lambda$的代数多重性大于1的体素需要跳过（忽略）；
- 只需检查体素所包含的点，是否位于同一平面时允许更大的方差，则能自然地扩展到非平面特征（BLAM只提到了平面特征和边缘特征）；
- 设置了两个条件来停止递归体素化：一个是树的最大深度，另一个是体素的最小点数。

<span style="color:red">正在持续更新中！</span>
### 八叉树数据结构

## 代码详解
## 参考文献
[1][《Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization》](https://arxiv.org/pdf/2109.06550)

[2][《BALM: Bundle Adjustment for Lidar Mapping》](https://www.arxiv.org/pdf/2010.08215)