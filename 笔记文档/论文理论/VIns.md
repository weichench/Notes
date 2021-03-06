



## VINSmono理论框架梳理

+ imu的数学模型，误差模型
+ 非线性最小二乘问题
  + 阻尼因子的迭代更新策略，阻尼因子的选取，鲁棒核函数（仅处理视觉的outlier）
+ VIO残差的构建
  + 基于逆深度的视觉重投影误差
  + imu的预计分误差，
    + 基于逆深度的视觉重投影误差imu的预积分，积分模型到预积分模型的转换
  + 预积分的方差
    + 方差的递推，协方差的传递
    + **根据每一步的观测，递推得到每步的误差均值，进一步推得每一步的协方差矩阵**
  + 视觉重投影的协防差矩阵？？？？视觉协方差可以人为设定
  + 重投影误差对状态量（位姿PQ，逆深度）的雅克比
  + imu预计分误差对状态量（PQV，两个bias）的雅克比
+ slam问题的概率建模，数学的角度，理解引入协方差矩阵
+ 协方差矩阵：变量两两之间的相关性（独立性） ，         信息矩阵：变量之间的条件概率
+ 舒尔补，得到边际（marg）掉变量后的信息矩阵
+ 为什么H矩阵(残差的信息矩阵乘雅克比，叠加)，是优化变量的信息矩阵？？
  + **叠加前的信息矩阵为单个矩阵块，叠加后，即组装为整体大的信息矩阵，整体信息矩阵的边际化**
+ FEJ算法，marg后，旧的先验，加上保留变量的观测残差
  + 先验信息部分，雅克比无法更新，线性化点固定
  + 两部分计算雅克比的线性化点必须一致，避免零空间的退化
+ 利用舒尔补，加速高斯牛顿方程的求解，计算得到变量的更新量
+ H矩阵不满秩，添加超强先验，或固定一个相机pose，限定优化值不乱飘
+ 先验残差的更新（迭代方程的右边），一阶泰勒的近似
+ ***滑动窗口中，当前帧若为非关键帧，则丢弃该帧的测量信息，imu的预计分传递至下一帧***
+ 金字塔光流，关键帧的选取策略，三角化
+ 初始化
  + 计相机与IMU之间的旋转，
  + 估计陀螺仪的bias，
  + 估计重力方向，速度，尺度的初始值，
  + 重力向量的进一步优化
  + 相机坐标系与世界坐标系的对齐





### 有关优化-使得整体的各马氏距离之和最小化

+ 两两帧之间，是让整体的误差最小化，（视觉与IMU预积分误差的马氏距离之和），从而得到待优化变量的迭代值，并非某单个误差的最小化
+ 滑动窗口的优化，同样是让滑窗内各误差的马氏距离之和最小化（各误差项的雅克比矩阵组装得到一个整体的H矩阵），得到各个待优化的状态量的迭代值（更新值）

## VINS原论文

### related work

1. 松耦合的文献
2. 基于EKF的紧耦合文献，，MSCKF
3. 基于滑动窗口的图优化，迭代求解的计算需求大，难实时
4. 直接法与非直接法
5. EKF中对imu数据的处理
6. IMU的预计分，旋转的协方差，但忽略了bias
7. 有关初始化，ORBslam的vio版本，初始化时间过长
8. 闭环限制，位姿4自由度

### 测量处理

1. LK光流追踪，新角点特征的提取,RANSAC外点剔除，关键帧的选择参考imu的运动估计与追踪到的特征点数目
2. 文献7,IMU协方差的传播，预积分，减去了bias，noise的处理？？
3. 预积分参考坐标的转换
4. imu,由预积分的误差均值的传递，推倒得到的误差协方差的传递？？？

### 初始化

+ 单目vins初始化时，最脆弱，松耦合的数据融合，获取初始化值，忽略加速度计的偏置（因为相对于重力，数值极小）
+ 纯视觉的sfm
  + 检测窗口内任意两帧的特征点跟踪超过30个，足够的视差，五点法恢复旋转与平移，设置任意尺度，三角化特征点，pnp估计窗口内其他帧的位姿，BA优化所有观测
  + 由相机相对于imu的初始外部参数（粗略值），将相机位姿转至imu系
+ 视觉得到的旋转，与imu预计分得到的旋转，误差最小化，求得陀螺仪的bias
+ 初始化速度，重力向量，尺度
+ 重力的细化

### 紧耦合的单目VIO

+ 协方差越大，表示测量噪声越大，该观测越不可靠，    意味着该IMU观测越不可信，换句话说，因IMU噪声较大，越不可信IMU预积分数据，而更加相信visual观测。注意，这里的IMU和visual协方差的绝对值没有意义，因为考虑得是两者的相对性。
+ Mahalanobis距离其实相当于一个残差加权，协方差大的加权小, 协方差小的加权大, 着重优化那些比较确定的残差
+ 视觉约束的噪声协方差与标定相机内参时的重投影误差，也就是偏离几个像素有关，代码对应为ProjectionTdFactor::sqrt_info，这里取的1.5个像素，对应到归一化相机平面上的协方差矩阵需除以焦距f，则信息矩阵等于协方差矩阵的逆，为：
+ **视觉重投影误差与imu预积分误差中，对应协方差矩阵的理解**
  + 重投影误差的协方差，表示了像素横纵坐标之间的相关性
  + imu预积分的协方差理解
    + 大的矩阵块，表示了不同多维变量之间的相关性，比如，位置误差与角度误差的相关性
      + 矩阵块内，各处的元素，表示了该两边量不同维度上的相关性，比如平移位置P的(x,y,z)与角度Q的(x,y,z,w)相互之间
+ 定义在单位球面上的视觉重投影误差
+ 基于舒尔补的边缘化
  + 方程两边都要做边缘化，**方程的右边也包含了信息矩阵,故需要边际化**
  + 当次新帧为关键帧时，MARGIN_OLD将marg掉最老帧，及其看到的路标点和相关联的IMU数据，将其转化为先验信息加到整体的目标函数中
  + 当次新帧不是关键帧时，MARGIN_SECOND_NEW，我们将直接扔掉次新帧及它的视觉观测边，而不对次新帧进行marg，因为我们认为当前帧和次新帧很相似，也就是说当前帧跟路标点之间的约束和次新帧与路标点的约束很接近，直接丢弃并不会造成整个约束关系丢失过多信息。但是值得注意的是，我们要保留次新帧的IMU数据，从而保证IMU预积分的连贯性
  + VINS没有使用FEJ算法
  + 每一关键帧都进入滑窗
+ 低计算平台下
  + 然而我们只对固定数量的最新IMU状态的姿态和速度进行了优化，而不是对滑动窗口中的所有状态进行优化。我们将特征深度、外部参数、偏置和旧的IMU状态这些不希望优化的状态作为常量来处理。
+ imu前向传递，以达到imu速率的状态估计
+ 故障的检测，不可避免，检测异常
  + 一旦检测到故障，切换会初始化，新建一个独立的位姿图

### 重定位

+ 闭环检测后，PNP求解相对位姿
  + pnp RANSAC求解基础矩阵，外点剔除，相对位姿存储，用于后续的闭环融合优化的初始值
+ 重定位——优化中添加回环观测，可以有多个回环观测
  + 闭环帧与新的关键帧，闭环链接
  + 将闭环帧加入到滑窗，闭环帧位姿固定（仅有视觉约束，无imu），作为一个额外的观测，将闭环帧与新关键帧的观测约束加入滑窗内的优化，
+ 添加被marg掉的关键帧到位姿图
  + 顺序边连接上一个关键帧，回环边链接回环帧，
  + 优化位姿图中所有的顺序边（历史关键帧）与回环边，论文fig6,45步，4自由度的全局位姿图优化
  + 随后，滑窗在优化过后的位姿图上进行重定位
+ 位姿图的管理：
  + 所有具有回环约束的关键帧都将被保留，而其他与相邻帧过近或方向非常相似的关键帧可能会被删除。关键帧被移除的概率和其相邻帧的空间密度成正比

### 实验分析

+ 我们的算法与OKVIS在许多细节上是不同的，如技术部分所示。我们的系统具有良好的初始化和回环功能
+ OKVIS在翻滚角和俯仰角估计方面表现更好。一个可能的原因是VINS-Mono采用了预积分技术，即IMU传递的一阶近似，以节省计算资源。**OKVIS对imu预计分的处理**
+ 对于纯VIO，VINS-Mono和OKVIS具有相似的精度，很难区分哪个比较好。然而，VINS-Mono在系统级别上优于OKVIS。它是一个完整的系统，具有鲁棒的初始化和回环闭合功能来辅助单目视觉。
+ 当我们在室内转圈时，会出现明显的漂移。OKVIS和只有VIO版本的VINS-Mono在x、y、z和偏航角上积累了大量漂移。我们的重定位和回环闭合模块有效地消除了所有这些漂移。

### 未来工作

+ 在线方法来评估单目VINS的可观测性
+ 用要求在线校准几乎所有传感器的内参和外参，以及在线鉴定校准质量
+ 作由单目VINS给出的稠密地图。我们在[44]中首次给出了用于无人机导航的单目视觉-惯性稠密地图的结果。然而，仍需进行广泛的研究以进一步提高系统的精度和鲁棒性。

## 需要进一步了解的

### 马氏距离与协方差的理解，参考网页         https://www.jianshu.com/p/5706a108a0c6

+ 马氏距离(刻画二维离分布的距离，去掉尺度，相关性的影响)
  + 不同变量之间的度量标准不一，数值大的相对更加分散，方差更大，马氏距离做方差的归一化，将椭圆形的多维变量分布，归一化为标准圆形，（比较不同数据集之间？？？）
  + 考虑到了变量之间的关联性，主成分分析，旋转坐标轴，旋转后的坐标轴线性无关，**为什么要考虑变量相关性的影响？？理解为：样本群的两个变量之间存在某种关联性规律，其用相关性表示，考虑某单个样本之间的相关规律对于样本群规律的符合程度**
    + 主成分分析，特征值与特征向量，特征向量表示了坐标轴旋转的方向，特征值表示了缩放的尺度，处理后，原分布变为正圆形
+ 协方差物理意义的理解
  + 表示变量相互关系的数字特征，![img](https://upload-images.jianshu.io/upload_images/14329037-8c447eeb2f3b1aa0.png?imageMogr2/auto-orient/strip|imageView2/2)
    + 大部分落在的1.3区域内，(X-EX)(Y-EY)>0,即为正相关;若大部分落在2,4区域，即为负相关，     若落在四个区域的概率一致，分布图为一个圆形，则为负相关
    + E[(X-EX)(Y-EY)]
    + 残差函数的协方差矩阵，表示了两两误差项之间的相关性
    + 协方差矩阵，两边量之间，与其他无关，   信息矩阵，条件相关性，固定其他变量
    + 协方差矩阵内元素的含义
      + 对角线元素为方差值，表示了分布的离散程度，越大，表示越离散
      + 当对角线元素非零，其他为0,表示两边量之间无相关性，分布的形状为圆或椭圆（不斜）
      + 非对角线非零，表示有相关性，为有倾斜的椭圆



+ 概率分布的函数，均值与方差，方差由均值推导得到？？?,与卡尔曼有关？？？
+ 理解马氏距离在最小二乘中的意义，总结马氏距离的应用场景
  + 当有多个距离需要计算时，考虑到变量与变量之间，变量内部各维度之间，尺度的不一致，变量之间存在相关性，马氏距离便能去除这些因素的影响，得到一个归一化的，去除掉相关性的客观距离
+ 正定矩阵？半正定矩阵？？
+ 



松耦合的文章在12～13年

msckf 2007年

VINS参考文献14,2013的滤波器文章

15年的文献18, 17年的文献19  是对msckf的扩展