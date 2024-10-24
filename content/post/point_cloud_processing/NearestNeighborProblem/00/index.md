---
title: "暴力最近邻法"
description: 最近邻问题系列
date: 2024-10-22T20:29:04+08:00
image: title.png
slug: NearestNeighborProblem/00
math: true
license: 
hidden: false
comments: true
draft: false
categories:
- PointCloudProcessing
- NearestNeighborProblem
tags:
- 最近邻问题
---
## 理论内容
暴力最近邻法(Brute-force Nearest Neighbour Search,BF 最近邻搜索)是最简单直观的最近邻计算方法,无须任何辅助的数据结构。
如果我们搜索一个点的最近邻,不妨称为暴力最近邻搜索;如果搜索k个最近邻,不妨称为暴力k近邻搜索。整体而言,这是一种简单粗暴的思路。

**暴力最近邻搜索**   给定点云$\mathcal{X}$和待查找点$x_m$,计算$x_m$与$\mathcal{X}中每个点的距离,并给出最小距离。
同理,可以类似地给出暴力k近邻的搜索方法:

**暴力k近邻（BF kNN）**
    
1. 对给定点云$\mathcal{X}$和查找点$x_m$,计算$x_m$对所有$\mathcal{X}$点的距离。
2. 对第1步的结果排序。
3. 选择$k$个最近的点。
4. 对所有$x_m$重复步骤1~3。
## 代码实践
1. 暴力最近邻

``` C++
//  单线程
/**
 * @brief Brute-force Nearest Neighbour
 * @param cloud 点云
 * @param point 待查找点
 * @return 找到的最近点索引
 */
int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    return std::min_element(cloud->points.begin(), cloud->points.end(),
        [&point](const PointType& pt1, const PointType& pt2) -> bool {
        return (pt1.getVector3fMap() - point).squaredNorm() <
              (pt2.getVector3fMap() - point).squaredNorm();
              }) -cloud->points.begin();
}
/**
 * @brief 对点云进行BF最近邻
 * @param cloud1  目标点云
 * @param cloud2  被查找点云
 * @param matches 两个点云内的匹配关系
 * @return
 */
 void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, 
                std::vector<std::pair<size_t, size_t>>& matches) {
    // 单线程版本
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    matches.resize(index.size());
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}
```
## 参考文献
[1][《自动驾驶与机器人中的 SLAM技术:从理论到实践》](https://product.dangdang.com/11478791697.html)

[2][《slam_in_autonomous_driving》——高翔老师的开源仓库](https://github.com/gaoxiang12/slam_in_autonomous_driving/tree/master)
