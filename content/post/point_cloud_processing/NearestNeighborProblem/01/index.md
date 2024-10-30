---
title: "K-d树"
description: 最近邻问题系列，摘录自高翔博士的《自动驾驶与机器人中的 SLAM技术:从理论到实践》
date: 2024-10-22T20:39:43+08:00
image: title.png
math: true
slug: NearestNeighborProblem/01
hidden: false
comments: true
categories:
- PointCloudProcessing
- NearestNeighborProblem
tags:
- 最近邻问题
---
## 理论内容
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;K-d树最早由Bentley Jon Louis提出,是二分树的高维度版本。
K-d 树也是二叉树的一种,任意一个 K-d 树的节点由左右两侧组成。在二分树里,可以用单个维度的信息来区分左右,但在 K-d 树里,由于要分割高维数据,会用超平面(Hyperplane)来区分左右侧(不过,对于三维点,实际上的超平面就是普通的二维平面)。在如何分割方面,则存在一些方法上的差异。下文介绍的是沿轴超平面分割(Axis-aligned Splitting Plane)。只需要沿着所有维度中任意一个轴将点云分开即可,实现起来十分简单。

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;我们可以对一个任意维度的点云建立 K-d 树,称为 K-d树的构建过程,或者叫建树。随后, 可以对空间中任意一点进行k近邻查找,称为查找过程。根据查找方法的不同,K-d树也分按范围查找(Search by Range)和按 k 最近邻查找(Search by k Nearest Neighbours),二者的具体实现方法大同小异。在K-d树中,以树状结构来表达点云的结构关系,规定:
1. 每个点云有左右两个分枝；
2. 叶子节点表示原始点云中的点。当然在实际存储时，可以存储点的索引而非点云本身，这样可以节省空间。
3. 非叶子节点存储一个分割轴和分割阈值，来表达如何分割左分枝和右分枝。例如，x=1可以存储为按第一个轴，阈值为1的方式来分割。规定左侧分枝取小于号，右侧分枝取大于等于号。

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;按照上述约定,就可以实现 K-d树的构建算法和查找算法。下面简单描述其算法步骤,然后给出实现和结果。由于 K-d 树在数据结构上还是一种树,所以大部分算法都可以用递归的形式很简洁地实现。
### K-d树的构建
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;在K-d树的构建过程中,主要考虑如何对给定点云进行分割。不同分割方法的策略不同。传统的做法,或是以固定顺序来交替坐标轴1099,或是计算当前点云在各轴上的分散程度,取分散程度最大的轴作为分割轴。这里介绍后一种方法

 K-d树的构建步骤如下:
1. 输入:点云数据 $X={x_1,\cdots,x_n}$,其中$x_i\in\mathbb{R}^{k}$。
2. 考虑将子集$X_n\subset X$插人节点$n$。
3. 如果$X_n$为空,则退出。
4. 如果$X_n$只有一个点,则记为叶子节点,退出。
5. 计算$X_n$在各轴的方差,挑选分布最大的一个轴,记为j;取平均数$m_j = X_n[j]$作为分割阈值。
6. 遍历 $x\in{X_n}$,对于$x[j]<m_j$的,插入左节点;否则插入右节点。
7. 递归上述步骤直到所有点都被插人树中。
### K-d树的查找

K-d 树的最近邻查找步骤如下：
1. 输人:K-d 树$T$,查找点$x$。
2. 输出:$x$的最近邻。
3. 记当前节点为$n_c$,最初取$n_c$为根节点。记$d$为当前搜索到的最小距离。\
(a) 如果$n_c$是叶子,则计算$n_c$与$x$的距离。看它是否小于$d$;若是,记$n_c$为最近邻,回退到它的父节点。\
(b) 如果$n_c$不是叶子,则计算$x$落在$n_c$的哪一侧。若$n_c$所在的一侧未被展开,则优先展开$n_c$所在的一侧。\
(c) 计算是否需要展开$n_c$的另一侧。记$x$与$n_c$分割面的距离为$d' $。若$d' <d$,则必须展开另一侧;否则跳过另一侧分枝。\
(d) 如果两侧都已经展开,或者不必展开,则返回上一节点,直到$n_c$变为根节点。

K-d树的最近邻查找步骤如下：
1. 输人:K-d树$T$,查找点$x$,最近邻数$k$。
2. 输出:k 近邻集合$ N$。
3. 记当前节点为$n_c$,最初取 $n_c$ 为根节点。令函数$S(n_c)$表示在 $n_c$ 下进行k近邻搜索: \
(a) 如果 $n_c$ 是叶子,则计算 $n_c$ 与 $x$ 的距离是否小于 $N$ 中的最大距离;若是,则将$n_c$放入N。若此时$|N|>k$,则删除 $N$ 中距离最大的匹配点。\
(b) 计算 $x$ 落在 $n_c$ 的哪一侧。递归调用 $S(n_c.left)$或 $S(n_c.right)$。\
(c) 计算是否需要展开$n_c$的另一侧。展开的条件判定:$|N|<k$时,必须展开;$|N|=k$且$x$与$n_c$的分割面距离小于$N$中最大匹配距离，也进行展开。\
(d) 若 $n_c$ 的令一侧不需要展开，则函数返回；否则继续调用另一侧的近邻搜索算法。
## 代码实践
``` C++
/// Kd树节点，二叉树结构，内部用祼指针，对外一个root的shared_ptr
struct KdTreeNode {
    int id_ = -1;
    int point_idx_ = 0;            // 点的索引
    int axis_index_ = 0;           // 分割轴
    float split_thresh_ = 0.0;     // 分割位置
    KdTreeNode* left_ = nullptr;   // 左子树
    KdTreeNode* right_ = nullptr;  // 右子树

    bool IsLeaf() const { return left_ == nullptr && right_ == nullptr; }  // 是否为叶子
};

/// 用于记录knn结果
struct NodeAndDistance {
    NodeAndDistance(KdTreeNode* node, float dis2) : node_(node), distance2_(dis2) {}
    KdTreeNode* node_ = nullptr;
    float distance2_ = 0;  // 平方距离，用于比较

    bool operator<(const NodeAndDistance& other) const { return distance2_ < other.distance2_; }
};



/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C    容器类型
 * @tparam D    结果类型
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    //通过getter获取每个索引对应的坐标，并计算均值与协方差
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}



class KdTree {
   public:
    explicit KdTree() = default;
    ~KdTree() { Clear(); }

    /**
     * 构建Kd树
     * @param cloud 点云
     */
    bool BuildTree(const CloudPtr& cloud){
        //初始化点云数据
           if (cloud->empty()) {
        return false;
    }

    cloud_.clear();
    cloud_.resize(cloud->size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_[i] = ToVec3f(cloud->points[i]);
    }
    //初始化
    Clear();
    Reset();

    IndexVec idx(cloud->size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        idx[i] = i;
    }

    Insert(idx, root_.get());
    return true; 
    }

    /// 获取k最近邻
    bool GetClosestPoint(const PointType& pt, std::vector<int>& closest_idx, int k = 5){
         if (k > size_) {
        LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
        return false;
    }
    k_ = k;

    std::priority_queue<NodeAndDistance> knn_result;
    Knn(ToVec3f(pt), root_.get(), knn_result);

    // 排序并返回结果
    closest_idx.resize(knn_result.size());
    for (int i = closest_idx.size() - 1; i >= 0; --i) {
        // 倒序插入
        closest_idx[i] = knn_result.top().node_->point_idx_;
        knn_result.pop();
    }
    return true;
    }  

    /// 并行为点云寻找最近邻
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5){
         matches.resize(cloud->size() * k);

        // 索引
        std::vector<int> index(cloud->size());
        for (int i = 0; i < cloud->points.size(); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx) {
            std::vector<int> closest_idx;
            GetClosestPoint(cloud->points[idx], closest_idx, k);
            for (int i = 0; i < k; ++i) {
                matches[idx * k + i].second = idx;
                if (i < closest_idx.size()) {
                    matches[idx * k + i].first = closest_idx[i];
                } else {
                    //NOTE: 非法定义 constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();
                    matches[idx * k + i].first = math::kINVALID_ID;
                }
            }
        });

        return true;
    }

    /// 这个被用于计算最近邻的倍数
    void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }

    /// 返回节点数量
    size_t size() const { return size_; }

    /// 清理数据
    void Clear(){
        for (const auto &np : nodes_) {
            if (np.second != root_.get()) {
                delete np.second;
            }
        }

        nodes_.clear();
        root_ = nullptr;
        size_ = 0;
        tree_node_id_ = 0;    
    }

    /// 打印所有节点信息
    void PrintAll(){
        for (const auto &np : nodes_) {
            auto node = np.second;
            if (node->left_ == nullptr && node->right_ == nullptr) {
                LOG(INFO) << "leaf node: " << node->id_ << ", idx: " << node->point_idx_;
            } else {
                LOG(INFO) << "node: " << node->id_ << ", axis: " << node->axis_index_ << ", th: " << node->split_thresh_;
            }
        }
    }

   private:
    /// kdtree 构建相关
    /**
     * 在node处插入点
     * @param points
     * @param node
     */
    void Insert(const IndexVec& points, KdTreeNode* node){
            nodes_.insert({node->id_, node});

    if (points.empty()) {
        return;
    }
    if (points.size() == 1) {
        size_++;
        node->point_idx_ = points[0];
        return;
    }
    //找到分割轴和阈值
    //NOTE:如果找不到分割轴，则将第一个点作为叶子节点，返回。这种情况可能是所有点都在一个位置上。
    IndexVec left, right;
    if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right)) {
        size_++;
        node->point_idx_ = points[0];
        return;
    }
    //递归插入
    const auto create_if_not_empty = [&node, this](KdTreeNode *&new_node, const IndexVec &index) {
        if (!index.empty()) {
            new_node = new KdTreeNode;
            new_node->id_ = tree_node_id_++;
            Insert(index, new_node);
        }
    };

    create_if_not_empty(node->left_, left);
    create_if_not_empty(node->right_, right);
    }

    /**
     * 计算点集的分割面
     * @param points 输入点云
     * @param axis   轴
     * @param th     阈值
     * @param left   左子树
     * @param right  右子树
     * @return
     */
    bool FindSplitAxisAndThresh(const IndexVec& point_idx, int& axis, float& th, IndexVec& left, IndexVec& right){
        // 计算三个轴上的散布情况，我们使用math_utils.h里的函数
    Vec3f var;
    Vec3f mean;
    math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx) { return cloud_[idx]; });//通过lambda表达式，将索引对应的点传给函数
    int max_i, max_j;
    var.maxCoeff(&max_i, &max_j);
    axis = max_i;//最大协方差轴
    th = mean[axis]; //阈值

    for (const auto &idx : point_idx) {
        if (cloud_[idx][axis] < th) {
            // 中位数可能向左取整
            left.emplace_back(idx);
        } else {
            right.emplace_back(idx);
        }
    }

    // 边界情况检查：输入的points等于同一个值，上面的判定是>=号，所以都进了右侧
    // 这种情况不需要继续展开，直接将当前节点设为叶子就行
    if (point_idx.size() > 1 && (left.empty() || right.empty())) {
        return false;
    }

    return true;
    }

    void Reset(){
        tree_node_id_ = 0;
        root_.reset(new KdTreeNode());
        root_->id_ = tree_node_id_++;
        size_ = 0;
    }

    /// 两个点的平方距离
    static inline float Dis2(const Vec3f& p1, const Vec3f& p2) { return (p1 - p2).squaredNorm(); }

    // Knn 相关
    /**
     * 检查给定点在kdtree node上的knn，可以递归调用
     * @param pt     查询点
     * @param node   kdtree 节点
     */
    void Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const{
           if (node->IsLeaf()) {
        // 如果是叶子，检查叶子是否能插入
        ComputeDisForLeaf(pt, node, knn_result);
        return;
    }

    // 看pt落在左还是右，优先搜索pt所在的子树
    // 然后再看另一侧子树是否需要搜索
    KdTreeNode *this_side, *that_side;
    if (pt[node->axis_index_] < node->split_thresh_) {
        this_side = node->left_;
        that_side = node->right_;
    } else {
        this_side = node->right_;
        that_side = node->left_;
    }

    Knn(pt, this_side, knn_result);
    //判断是否需要展开
    if (NeedExpand(pt, node, knn_result)) {  // 注意这里是跟自己比
        Knn(pt, that_side, knn_result);
    }
    }

    /**
     * 对叶子节点，计算它和查询点的距离，尝试放入结果中
     * @param pt    查询点
     * @param node  Kdtree 节点
     */
    void ComputeDisForLeaf(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const{
            // 比较与结果队列的差异，如果优于最远距离，则插入
    float dis2 = Dis2(pt, cloud_[node->point_idx_]);
    if (knn_result.size() < k_) {
        // results 不足k
        knn_result.emplace(node, dis2);
    } else {
        // results等于k，比较current与max_dis_iter之间的差异
        if (dis2 < knn_result.top().distance2_) {
            knn_result.emplace(node, dis2);
            knn_result.pop();
        }
    }
    }

    /**
     * 检查node下是否需要展开
     * @param pt   查询点
     * @param node Kdtree 节点
     * @return true if 需要展开
     */
    bool NeedExpand(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const{
            if (knn_result.size() < k_) {
        return true;
    }
    //前面也说过,判断另一侧的是否展开的条件是判断当前找到最小的距离是否小于到分界线的距离，
    //如果是，则说明另一侧的点肯定是更远的，则不用展开
    if (approximate_) {
        float d = pt[node->axis_index_] - node->split_thresh_;
        if ((d * d) < knn_result.top().distance2_ * alpha_) {
            return true;
        } else {
            return false;
        }
    } else {
        // 检测切面距离，看是否有比现在更小的
        float d = pt[node->axis_index_] - node->split_thresh_;
        if ((d * d) < knn_result.top().distance2_) {
            return true;
        } else {
            return false;
        }
    }
    }

    int k_ = 5;                                   // knn最近邻数量
    std::shared_ptr<KdTreeNode> root_ = nullptr;  // 根节点
    std::vector<Vec3f> cloud_;                    // 输入点云
    std::unordered_map<int, KdTreeNode*> nodes_;  // for bookkeeping

    size_t size_ = 0;       // 叶子节点数量
    int tree_node_id_ = 0;  // 为kdtree node 分配id

    // 近似最近邻
    bool approximate_ = true;
    float alpha_ = 0.1;
};

```
<span style="color:red;">遗留内容：如何删除K-d树中的一个点，如何对K-d树进行平衡。</span>
## 参考文献
[1][《自动驾驶与机器人中的 SLAM技术:从理论到实践》](https://product.dangdang.com/11478791697.html)

[2][《slam_in_autonomous_driving》——高翔老师的开源仓库](https://github.com/gaoxiang12/slam_in_autonomous_driving/tree/master)
