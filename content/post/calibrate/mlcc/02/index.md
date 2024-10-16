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
draft: false
categories:
- Calibrate
tags:
- mlcc
---
## 文章内容
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;With the LiDAR extrinsic parameter $\mathcal{E}_L$  and pose trajectory $\mathcal{S}$ computed above, we obtain a dense global point cloud by transforming all LiDAR points to the base LiDAR frame. Then, the extrinsic $\mathcal{E}_C$ is optimized by minimizing the summed distance between the back-projected LiDAR edge feature points and the image edge feature points. Two types of LiDAR edge points could be extracted from the point cloud. One is the depth-discontinuous edge between the foreground and background objects, and the other is the depth-continuous edge between two neighboring non-parallel planes. As explained in our previous work [28], depth-discontinuous edges suffer from foreground inflation and bleeding points phenomenon; we hence use depth-continuous edges to match the point cloud and images.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;利用上述计算得到的激光雷达外参参数 $\mathcal{E}_L$ 和位姿轨迹 $\mathcal{S}$，我们将所有激光雷达点转换到基准激光雷达坐标系中，从而获得一个密集的全局点云。接着，通过最小化反向投影的激光雷达边缘特征点与图像边缘特征点之间的累积距离来优化相机外参 $\mathcal{E}_C$。可以从点云中提取两种类型的激光雷达边缘点。一种是前景和背景物体之间的深度不连续边缘；另一种是非平行邻近平面之间的深度连续边缘。正如我们先前的工作 [28] 中所解释的那样，深度不连续边缘容易出现前景膨胀和漂移点现象；因此，我们使用深度连续边缘来进行点云与图像的匹配。

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

论文中相机标定部分的内容是比较容易理解的，只需找到匹配的点云边缘点和图像边缘点，将点云边缘点投影到图像上，通过
最小化重投影误差即可对相机外参进行优化，其中最重要的是作者建立边缘线上点云与图像的匹配点的实现思路。至于最后的
误差公式的一阶导数推导，暂不进行说明，因为代码实现中直接使用了ceres的自动求导。<span style="color:yellow">
后续有空余时间可以补充！</span>

建立匹配点的思路为：将所有点云边缘线上全局坐标系下的点云投影到基准雷达下(注意，是所有位姿都投影，而非对应的某
一帧，因为不同图像可能会看到同一个边缘线)，再通过cv::projectPoints()函数直接投到像素坐标系下，遍历每个边缘
点，判断其距离最近的5个图像上的边缘点，如果5个点距离该点都小于一定的值则检索该点距离最近的5个点，拟合点云的方向
向量，图像也是一样的处理，将那5个点拟合方向向量。需要注意的是图像的分辨率是有限的，尤其是在较低分辨率的相机中，多个三维点
可能在投影到二维平面后，落在同一个像素上。因此那些投影之后落在同一个位置的三维点要取平均值。此时所有的三维点都将有
其对应的二维点。

## 代码详解
### 平面处理
1. 体素化提取空间中的平面

下面代码选自ba.hpp,通过判断特征值的比值确认平面，核心内容与之前的雷达标定部分是一样的，但是多了一些细节处理，就是将平面拆分成8份后，将每一份的法向量与整体的法向量进行一致性评估。具体如下：
``` C++

  bool judge_eigen(int layer) {
    VOX_FACTOR covMat = sig_orig;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat.cov());
    value_vector = saes.eigenvalues();
    center = covMat.v / covMat.N;//平面中心点
    direct = saes.eigenvectors().col(0);//平面的法向量

    eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2]; // [0] is the smallest
    //NOTE:与雷达标定部分检测平面刚好相反，这里是小值比大值。
    if (eigen_ratio > eigen_thr) return 0;
    //NOTE:两个较小的特征值比值如果比较接近，那么说明在剩余两个方向分部比较均匀，是线状点云
    if (saes.eigenvalues()[0] / saes.eigenvalues()[1] > 0.1) return 0; // 排除线状点云

    double eva0 = saes.eigenvalues()[0];
    double sqr_eva0 = sqrt(eva0);
    //NOTE:选择平面中心点沿着平面法向量的方向延伸一定的距离的点作为边界点
    Eigen::Vector3d center_turb = center + 5 * sqr_eva0 * direct;
    //NOTE:将平面拆分为8个子平面
    vector<VOX_FACTOR> covMats(8);

    for (Eigen::Vector3d ap : vec_orig) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (ap(k) > center_turb[k])
          xyz[k] = 1;

      Eigen::Vector3d pvec(ap(0), ap(1), ap(2));

      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      covMats[leafnum].push(pvec);
    }

    //NOTE:重新计算子平面的法向量，判断与平面法向量的夹角是否足够小cos(θ)，当所有的都满足，则认为该平面是平面，否则不是平面
    int num_all = 0, num_qua = 0;
    for (int i = 0; i < 8; i++)
      if (covMats[i].N > MIN_PT) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMats[i].cov());
        Eigen::Vector3d child_direct = saes.eigenvectors().col(0);

        if (fabs(child_direct.dot(direct)) > 0.98)
          num_qua++;
        num_all++;
      }
    if (num_qua != num_all) return 0;
    return 1;
  }

```
2. 平面整合
选自calib_camera.hpp，将体素中属于同一平面的点云进行合并，判断条件是两个平面法向量相似性很高，且彼此法向量距离对方的平面中心点距离很近。
``` C++
 /**
   * @brief 合并平面
   * @param origin_list 原始平面列表
   * @param merge_list 合并后的平面列表
   */
  void mergePlane(std::vector<Plane *> &origin_list, std::vector<Plane *> &merge_list) {
    for (size_t i = 0; i < origin_list.size(); i++)
      origin_list[i]->id = 0;

    int current_id = 1;
    for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--) {
      for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++) {
        //NOTE:计算当前平面和其他平面之间的法向量的差和法向量的和，以及平面中心到平面的距离，
        // 如果满足条件则认为两个平面是同一个平面，方向相同，距离接近，非常有可能为同一平面
        Eigen::Vector3d normal_diff = (*iter)->normal - (*iter2)->normal;
        Eigen::Vector3d normal_add = (*iter)->normal + (*iter2)->normal;
        double dis1 = fabs((*iter)->normal(0) * (*iter2)->center(0) +
            (*iter)->normal(1) * (*iter2)->center(1) +
            (*iter)->normal(2) * (*iter2)->center(2) + (*iter)->d);
        double dis2 = fabs((*iter2)->normal(0) * (*iter)->center(0) +
            (*iter2)->normal(1) * (*iter)->center(1) +
            (*iter2)->normal(2) * (*iter)->center(2) + (*iter2)->d);
        if (normal_diff.norm() < 0.2 || normal_add.norm() < 0.2)
          if (dis1 < 0.05 && dis2 < 0.05) {
            if ((*iter)->id == 0 && (*iter2)->id == 0) {
              (*iter)->id = current_id;
              (*iter2)->id = current_id;
              current_id++;
            } else if ((*iter)->id == 0 && (*iter2)->id != 0)
              (*iter)->id = (*iter2)->id;
            else if ((*iter)->id != 0 && (*iter2)->id == 0)
              (*iter2)->id = (*iter)->id;
          }
      }
    }
    .....
  }
```
### 平面交线（边缘线）处理
1. 点云边缘线处理

首先是通过两个平面法向量和中心点，计算交线上一点。假设$c_1$和$c_2$分别为平面1和2的中心点，$n_1$和$n_2$分别为平面1和2的法向量。而通过向量的叉乘很容易得到两个平面相交直线的法向量为$d=n1\times{n2}$。通过以下三个方程可以求得交线上的一点坐标$p$:
$$
\begin{align*}
n_1\cdot(p-c_1)=0\tag{1}\\
n_2\cdot(p-c_2)=0\tag{2}\\
d\cdot(p-c_1)=0\tag{3}
\end{align*}
$$
其中1式含义为交线上一点与平面1中心点$c_1$构成的向量垂直于$n_1$,式2同理，式3的含义为交线上一点与平面1中心点$c_1$构成的向量垂直于直线。因此我们设定：
$$
\begin{align*}
A=\begin{bmatrix}
n_1^{T}\\d^{T}\\n_2^{T}
\end{bmatrix}\tag{4}\\
b=\begin{bmatrix}n_1\cdot{c_1}\quad{d}\cdot{c_1}\quad{n_2}\cdot{c_2}\end{bmatrix}\tag{5}\\
A\cdot{x}=b\tag{6}
\end{align*}
$$
对式6进行QR分解则能得到交线上一点，且该点与$c1$构成的向量是垂直于交线的。另外：
$$
\begin{align*}
(c_2-O)\cdot{d}\times{d}\tag{7}\\
(c_2-0)-(c_2-O)\cdot{d}\times{d}\tag{8}
\end{align*}
$$
式7为$c_2$到交线投影位置与$O$点构成的向量,式8则为$c_2$与直线上一点构成垂直于交线的向量；
代码如下所示
``` C++
void projectLine(const Plane *plane1, const Plane *plane2,
                   std::vector<Eigen::Vector3d> &line_point) {
    float theta = plane1->normal.dot(plane2->normal);
    //夹角要大于一定的值
    if (!(theta > theta_max_ && theta < theta_min_)) return;
//    std::cout << "theta:" << theta << std::endl;
//    std::cout << "theta_max_" << theta_max_ << " theta_min_" << theta_min_ << std::endl;
    Eigen::Vector3d c1 = plane1->center;
    Eigen::Vector3d c2 = plane2->center;
    Eigen::Vector3d n1 = plane1->normal;
    Eigen::Vector3d n2 = plane2->normal;

    Eigen::Matrix3d A;
    Eigen::Vector3d d = n1.cross(n2).normalized();
    A.row(0) = n1.transpose();
    A.row(1) = d.transpose();
    A.row(2) = n2.transpose();
    //NOTE:描述了三个关系 $n_1\cdot{(p-c_1)}=0$ 、$n_2\cdot{(p-c_2)}=0$、$ d\cdot{(p-c_1)}$
    //该点与c1平面内中心点构成的向量与n1垂直，与c2平面中心点构成的向量与n2垂直，与c1平面中心点构成·与两个平面法向量、
    //垂直方向的法向量构成的向量垂直，所以该点为平面交线上一点
    Eigen::Vector3d b(n1.dot(c1), d.dot(c1), n2.dot(c2));
    Eigen::Vector3d O = A.colPivHouseholderQr().solve(b);

    double c1_to_line = (c1 - O).norm();
    //NOTE:注意计算点的时候约束了，该点与c1构成的向量是垂直于d的，但是没约束c2
    double c2_to_line = ((c2 - O) - (c2 - O).dot(d) * d).norm();

    if (c1_to_line / c2_to_line > 8 || c2_to_line / c1_to_line > 8) return;

    if (plane1->points_size < plane2->points_size)
      for (auto pt : plane1->plane_points) {
        Eigen::Vector3d p = (pt - O).dot(d) * d + O;
        line_point.push_back(p);
      }
    else
      for (auto pt : plane2->plane_points) {
        Eigen::Vector3d p = (pt - O).dot(d) * d + O;
        line_point.push_back(p);
      }

    return;
  }
```
进一步处理，计算在体素内所有点中距离平面投影线上的点最近的5个点，将其存放到相应的容器中。
``` C++
/**
   * @brief 提取体素地图中平面的边缘点
   * @param surf_map
   */
  void estimate_edge(std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> &surf_map) {
//    ros::Rate loop(500);
    lidar_edge_clouds = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++) {
      std::vector<Plane *> plane_list;
      std::vector<Plane *> merge_plane_list;
      iter->second->get_plane_list(plane_list);

      if (plane_list.size() > 1) {
        pcl::KdTreeFLANN<pcl::PointXYZI> kd_tree;
        pcl::PointCloud<pcl::PointXYZI> input_cloud;
        //将当前所有体素的点云加入到kd树中，用于快速最临近搜索
        for (auto pv : iter->second->all_points) {
          pcl::PointXYZI p;
          p.x = pv(0);
          p.y = pv(1);
          p.z = pv(2);
          input_cloud.push_back(p);
        }
        kd_tree.setInputCloud(input_cloud.makeShared());
        mergePlane(plane_list, merge_plane_list);
        if (merge_plane_list.size() <= 1) continue;
#ifdef DEBUG
        for (auto plane : merge_plane_list) {
          static int i = 0;
          pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
          std::vector<unsigned int> colors;
          colors.push_back(static_cast<unsigned int>(rand() % 255));
          colors.push_back(static_cast<unsigned int>(rand() % 255));
          colors.push_back(static_cast<unsigned int>(rand() % 255));
          for (auto pv : plane->plane_points) {
            pcl::PointXYZRGB pi;
            pi.x = pv[0];
            pi.y = pv[1];
            pi.z = pv[2];
            pi.r = colors[0];
            pi.g = colors[1];
            pi.b = colors[2];
            color_cloud.points.push_back(pi);
          }
          pcl::io::savePCDFile("merge_plane_" + std::to_string(i++)+".pcd", color_cloud);
        }
#endif
        for (size_t p1_index = 0; p1_index < merge_plane_list.size() - 1; p1_index++)
          for (size_t p2_index = p1_index + 1; p2_index < merge_plane_list.size(); p2_index++) {
            std::vector<Eigen::Vector3d> line_point;
            //计算两个平面之间的交线
            projectLine(merge_plane_list[p1_index], merge_plane_list[p2_index], line_point);

            if (line_point.size() == 0) break;

            pcl::PointCloud<pcl::PointXYZI> line_cloud;

            for (size_t j = 0; j < line_point.size(); j++) {
              pcl::PointXYZI p;
              p.x = line_point[j][0];
              p.y = line_point[j][1];
              p.z = line_point[j][2];

              int K = 5;
              // 创建两个向量，分别存放近邻的索引值、近邻的中心距
              std::vector<int> pointIdxNKNSearch(K);
              std::vector<float> pointNKNSquaredDistance(K);
              if (kd_tree.nearestKSearch(p, K, pointIdxNKNSearch, pointNKNSquaredDistance) == K) {
                Eigen::Vector3d tmp(input_cloud.points[pointIdxNKNSearch[K - 1]].x,
                                    input_cloud.points[pointIdxNKNSearch[K - 1]].y,
                                    input_cloud.points[pointIdxNKNSearch[K - 1]].z);
                // if(pointNKNSquaredDistance[K-1] < 0.01)
                if ((tmp - line_point[j]).norm() < 0.05) {
                  line_cloud.points.push_back(p);
                  lidar_edge_clouds->points.push_back(p);
                }
              }
            }
          }
      }
    }
  }
```
2. 图像交线处理
代码主要是通过opencv的cv::Canny()函数实现，没有太多额外的处理，有以下几点需要关注：
``` C++
    ...
 //高斯模糊，减小图像中的噪声和细节，保留图像的边缘
cv::GaussianBlur(src_img[a], src_img[a], cv::Size(gaussian_size, gaussian_size), 0, 0);
cv::Mat canny_result = cv::Mat::zeros(src_img[a].rows, src_img[a].cols, CV_8UC1);
//低于阈值1的像素点会被认为不是边缘；
//高于阈值2的像素点会被认为是边缘；
//在阈值1和阈值2之间的像素点,若与第2步得到的边缘像素点相邻，则被认为是边缘，否则被认为不是边缘。
cv::Canny(src_img[a], canny_result, canny_threshold, canny_threshold * 3, 3, true);
    ...
```
### 构建匹配对
本文第2章理论推导中对建立匹配对的过程进行了一定的描述，这里不再复述。
``` C++
  void buildVPnp(const Camera &cam,
                 const Vector6d &extrinsic_params, const int dis_threshold,
                 const bool show_residual,
                 const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cam_edge_clouds_2d,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_edge_clouds_3d,
                 std::vector<VPnPData> &pnp_list) {
    pnp_list.clear();
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << cam.fx_, cam.s_, cam.cx_, 0.0, cam.fy_, cam.cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff =
        (cv::Mat_<double>(1, 5) << cam.k1_, cam.k2_, cam.p1_, cam.p2_, cam.k3_);
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 =
        Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_(rotation_vector3);
    for (size_t a = 0; a < base_poses.size(); a += 1)  // for each camera pose
    {
      std::vector<std::vector<std::vector<pcl::PointXYZI>>> img_pts_container;
      for (int y = 0; y < cam.height_; y++) {
        std::vector<std::vector<pcl::PointXYZI>> row_pts_container;
        for (int x = 0; x < cam.width_; x++) {
          std::vector<pcl::PointXYZI> col_pts_container;
          row_pts_container.push_back(col_pts_container);
        }
        img_pts_container.push_back(row_pts_container);
      }
      std::vector<cv::Point3f> pts_3d;
      std::vector<cv::Point2f> pts_2d;
      cv::Mat r_vec = (cv::Mat_<double>(3, 1)
          << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
          rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
          rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
      Eigen::Vector3d t_(extrinsic_params[3], extrinsic_params[4], extrinsic_params[5]);
      cv::Mat t_vec = (cv::Mat_<double>(3, 1) << t_(0), t_(1), t_(2));

      for (size_t i = 0; i < lidar_edge_clouds_3d->size(); i++) {
        pcl::PointXYZI point_3d = lidar_edge_clouds_3d->points[i];
        Eigen::Vector3d pt1(point_3d.x, point_3d.y, point_3d.z);
        Eigen::Vector3d pt2(0, 0, 1);
        Eigen::Vector3d pt;
        pt = base_poses[a].q.inverse() * (pt1 - base_poses[a].t);//转回到基准雷达坐标系下
        //NOTE:将点转化到相机坐标系下，其与原点构成的向量与相机视野方向向量（0,0,1）夹角很小则说明是在相机视野内的点
        if (cos_angle(q_ * pt + t_, pt2) > 0.8) // FoV check
          pts_3d.emplace_back(cv::Point3f(pt(0), pt(1), pt(2)));
      }
      //将雷达点投到图像上
      cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);

      pcl::PointCloud<pcl::PointXYZ>::Ptr line_edge_cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<int> line_edge_cloud_2d_number;
      for (size_t i = 0; i < pts_2d.size(); i++) {
        pcl::PointXYZ p;
        p.x = pts_2d[i].x;
        p.y = -pts_2d[i].y;
        p.z = 0;
        pcl::PointXYZI pi_3d;
        pi_3d.x = pts_3d[i].x;
        pi_3d.y = pts_3d[i].y;
        pi_3d.z = pts_3d[i].z;
        pi_3d.intensity = 1;
        //判断是否在图像视野内
        if (p.x > 0 && p.x < cam.width_ && pts_2d[i].y > 0 && pts_2d[i].y < cam.height_) {
          if (img_pts_container[pts_2d[i].y][pts_2d[i].x].size() == 0) {
            line_edge_cloud_2d->points.push_back(p);
            img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
          } else
            img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
        }
      }
      if (show_residual)
        if (a == 16) {
          cv::Mat residual_img = getConnectImg(
              cam, dis_threshold, cam_edge_clouds_2d[a], line_edge_cloud_2d);
          std::string img_name = std::to_string(a);
          cv::imshow(img_name, residual_img);
          cv::waitKey(10);
        }

      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_cam(new pcl::search::KdTree<pcl::PointXYZ>());
      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_lidar(new pcl::search::KdTree<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud =
          pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_cam =
          pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_lidar =
          pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      kdtree_cam->setInputCloud(cam_edge_clouds_2d[a]);
      kdtree_lidar->setInputCloud(line_edge_cloud_2d);
      tree_cloud_cam = cam_edge_clouds_2d[a];
      tree_cloud_lidar = line_edge_cloud_2d;
      search_cloud = line_edge_cloud_2d;

      int K = 5; // 指定近邻个数
      // 创建两个向量，分别存放近邻的索引值、近邻的中心距
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      std::vector<int> pointIdxNKNSearchLidar(K);
      std::vector<float> pointNKNSquaredDistanceLidar(K);
      std::vector<cv::Point2d> lidar_2d_list;
      std::vector<cv::Point2d> img_2d_list;
      std::vector<Eigen::Vector2d> camera_direction_list;
      std::vector<Eigen::Vector2d> lidar_direction_list;
      std::vector<int> lidar_2d_number;
      for (size_t i = 0; i < search_cloud->points.size(); i++) {
        pcl::PointXYZ searchPoint = search_cloud->points[i];
        //查找点云的临近点，主要目的是为了计算点所在直线的方向
        kdtree_lidar->nearestKSearch(searchPoint, K, pointIdxNKNSearchLidar,
                                     pointNKNSquaredDistanceLidar);
        if (kdtree_cam->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
          bool dis_check = true;
          //如果点云中某个点与图像中最临近的5个点，有一个距离超出阈值，则认为该点与图像中该点距离太远，丢弃
          for (int j = 0; j < K; j++) {
            float distance =
                sqrt(pow(searchPoint.x - tree_cloud_cam->points[pointIdxNKNSearch[j]].x, 2) +
                    pow(searchPoint.y - tree_cloud_cam->points[pointIdxNKNSearch[j]].y, 2));
            if (distance > dis_threshold) dis_check = false;
          }
          if (dis_check) {
            cv::Point p_l_2d(search_cloud->points[i].x, -search_cloud->points[i].y);
            cv::Point p_c_2d(tree_cloud_cam->points[pointIdxNKNSearch[0]].x,
                             -tree_cloud_cam->points[pointIdxNKNSearch[0]].y);
            Eigen::Vector2d direction_cam(0, 0);
            std::vector<Eigen::Vector2d> points_cam;
            for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
              Eigen::Vector2d p(tree_cloud_cam->points[pointIdxNKNSearch[i]].x,
                                -tree_cloud_cam->points[pointIdxNKNSearch[i]].y);
              points_cam.push_back(p);
            }
            //计算点的分布方向，即直线方向，与计算通过协方差的特征值判断平面法向量类似
            calcDirection(points_cam, direction_cam);

            Eigen::Vector2d direction_lidar(0, 0);
            std::vector<Eigen::Vector2d> points_lidar;
            for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
              Eigen::Vector2d p(tree_cloud_lidar->points[pointIdxNKNSearchLidar[i]].x,
                                -tree_cloud_lidar->points[pointIdxNKNSearchLidar[i]].y);
              points_lidar.push_back(p);
            }
            calcDirection(points_lidar, direction_lidar);

            if (p_l_2d.x > 0 && p_l_2d.x < cam.width_ && p_l_2d.y > 0 &&
                p_l_2d.y < cam.height_) {
              lidar_2d_list.push_back(p_l_2d);
              img_2d_list.push_back(p_c_2d);
              camera_direction_list.push_back(direction_cam);
              lidar_direction_list.push_back(direction_lidar);
            }
          }
        }
      }
      for (size_t i = 0; i < lidar_2d_list.size(); i++) {
        int y = lidar_2d_list[i].y;
        int x = lidar_2d_list[i].x;
        int pixel_points_size = img_pts_container[y][x].size();
        if (pixel_points_size > 0) {
          VPnPData pnp;
          pnp.x = 0;
          pnp.y = 0;
          pnp.z = 0;
          pnp.u = img_2d_list[i].x;
          pnp.v = img_2d_list[i].y;
          //NOTE:图像的分辨率是有限的，尤其是在较低分辨率的相机中，多个三维点可能在投影到二维平面后，落在同一个像素上。
          for (int j = 0; j < pixel_points_size; j++) {
            pnp.x += img_pts_container[y][x][j].x;
            pnp.y += img_pts_container[y][x][j].y;
            pnp.z += img_pts_container[y][x][j].z;
          }
          pnp.x = pnp.x / pixel_points_size;
          pnp.y = pnp.y / pixel_points_size;
          pnp.z = pnp.z / pixel_points_size;
          pnp.direction = camera_direction_list[i];
          pnp.direction_lidar = lidar_direction_list[i];
          pnp.number = 0;
          float theta = pnp.direction.dot(pnp.direction_lidar);
          // 判断两个方向夹角是否满足条件
          if (theta > direction_theta_min_ || theta < direction_theta_max_)
            pnp_list.push_back(pnp);
        }
      }
    }
  }
```
<span style="color:red">正在更新中...</span>

## 参考文献
[1][《Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization》](https://arxiv.org/pdf/2109.06550)

[2][《BALM: Bundle Adjustment for Lidar Mapping》](https://www.arxiv.org/pdf/2010.08215)
