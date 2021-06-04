```

roslaunch vins_estimator tum.launch
roslaunch vins_estimator vins_rviz.launch

  rosrun vins vins_node /home/slam/catkin_ws_6/src/VINS-Fusion-master/config/tum/tum_stereo_imu_config.yaml    双目直接跑飞 (跳了3秒)
  
   rosrun vins vins_node ~/catkin_ws_2/src/git_project/VINS-Fusion/config/tum/tum_stereo_imu_config.yaml 

evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum -as -va --plot 

evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum -as -va --plot 


evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum  /home/slam/catkin_ws_t/src/git_project/VINS-Fusion/vio.tum -as -va --plot 

evo_traj tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum  --ref=/home/slam/catkin_ws_t/src/git_project/VINS-Fusion/vio.tum -as -va --plot

vins_mono   rmse	0.286480
双目运行的结果 rmse  0.30966
0.23
直接使用单目初始化得到的bias      





evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum /home/slam/catkin_ws_2/src/git_project/PL-VIO-master/config/traj/vins_result.tum  -as -va --plot 
rmse	0.219919


evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum /home/slam/catkin_ws_6/src/VINS-Fusion-master/vio.tum  -as -va --plot 



/home/slam/Dataset/MH_03_medium/mav0/state_groundtruth_estimate0

evo_ape tum /home/slam/Dataset/MH_03_medium/mav0/state_groundtruth_estimate0/data_.tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum -as  -va --plot 



evo_traj tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum  --plot
evo_traj tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum   --plot


```



____________

## 系统的改动

1. 结构线的辨识应该放在主节点
2. 结构化场景的探测一帧上进行即可

## 总述

+ 原本是在每帧上提取消失点并辨识结构线。每一条辨识到的结构线有一个与之对应的消失点。三角化时，根据相机系下消失点的方向，再判断该直线与Rws的哪个坐标轴对齐。
+ 现在改成了先得到一个唯一的Rws，根据Rws转到当前最新的相机系下，直接辨识出直线是对齐于Rws的哪个坐标轴。具体的程序改动就是把结构线的辨识及三角化放在了当前帧位姿计算以后。
+ 结构线辨识时的角度阈值可能会对结果有影响

_______

## 消失点

1. 红线是数量最多的一组平行线，在红绿与红蓝之间根据90度的夹角确定一个平面

_______

**最重要的还是要脑子清楚，想明白，理清楚。**

**糊里糊涂的套用，画模板，往往出错**



_________

## 存在的问题

+ check_and_visualize() 检查最新帧上收到的线特征及机构线的辨识情况
  + init_vp_flag， 最新帧上观测到线的结构线辨识情况
  + 最新帧上显示L_frame_flag(表示已经在之前三角化了的)，线数量少，表示最新帧上受到的线的测量约束其实不很多。。。。**线的观测生命首期太短了**
  + 当前帧上观测到的特征数量与建立位姿约束的特征数量作为评判依据



### vins-mono IMU初始化

1. pnp位姿求解
2. 单目初始化得到的结果
3. 单目初始化前仅以C0作为世界系，初始化以后，再进行重力对齐。双目版本直接进行重力对齐。
4. 只有平移足够，才能计算出尺度。双目版本的程序中，本身已经放到米制单位下的平移就很小，所以在尺度的估算中，数值不稳定，极易出错。——所以直接双目的时候，就不应该再有估算尺度这一步？
5. 所以，初始化以前，应该先纯视觉优化，然后估计IMU的bias及重力向量。不再估算尺度



current frame_count 10
init gravity0 0 0
all_image_frame states: 
size of all_image_frame11
frame 0 T:-0 -0 -0
frame 1 T: -0.132982  0.0452411 -0.0967227
frame 2 T:-0.171144 0.0501151 -0.301557
frame 3 T:-0.144959  0.066908 -0.480024
frame 4 T: -0.0807975 -0.00767739    -0.63192
frame 5 T: 0.0144877 -0.0597263  -0.795442
frame 6 T: 0.0505399 -0.0190777  -0.901574
frame 7 T:  0.142712 -0.0138167  -0.923034
frame 8 T:  0.244475 0.00923192  -0.902839
frame 9 T: 0.357765 0.0640329 -0.903679
[ WARN] [1610435940.391399224]: gyroscope bias initial calibration 0.00208848 0.00123568 0.00355482
Bgs0: 0.00208848 0.00123568 0.00355482
Bgs1: 0.00208848 0.00123568 0.00355482
Bgs2: 0.00208848 0.00123568 0.00355482
Bgs3: 0.00208848 0.00123568 0.00355482
Bgs4: 0.00208848 0.00123568 0.00355482
Bgs5: 0.00208848 0.00123568 0.00355482
Bgs6: 0.00208848 0.00123568 0.00355482
Bgs7: 0.00208848 0.00123568 0.00355482
Bgs8: 0.00208848 0.00123568 0.00355482
Bgs9: 0.00208848 0.00123568 0.00355482
Bgs10: 0.00208848 0.00123568 0.00355482
[DEBUG] [1610435940.393497795]: estimated scale: 0.020256
[DEBUG] [1610435940.393523184]:  result g     9.78537 -0.203815  -9.78161  0.179382
parameters G:       0       0 9.80766
[DEBUG] [1610435940.393693167]:  refine     9.80766 -0.22087 -9.80386 0.160347
refined Bgs[0]: 0.00208848 0.00123568 0.00355482
refined Bgs[10]: 0.00208848 0.00123568 0.00355482
refined gravity: -0.22087 -9.80386 0.160347
refined scale:   0.0303685   0.0157984   0.0157402   0.0278032   0.0435536  0.00985283 -0.00681543   0.0294722 -0.00550763  -0.0151636   0.0358953  0.00104963  -0.0347468   0.0324523   0.0200264  -0.0201995   0.0278505   0.0020407  -0.0197142  0.00264894 -0.00432964  -0.0126433 -0.00386416 -0.00336789  -0.0095905 -0.00254645  -0.0124267  -0.0186025  -0.0013351  -0.0233946  -0.0278384  0.00678363 -0.00748465 -0.00486028  0.00414573   0.0222994
s: 0.0222994
Vs0: 0.0303685 0.0157984 0.0157402
Vs1:  0.0278032  0.0435536 0.00985283
Vs2: -0.00681543   0.0294722 -0.00550763
Vs3: -0.0151636  0.0358953 0.00104963
Vs4: -0.0347468  0.0324523  0.0200264
Vs5: -0.0201995  0.0278505  0.0020407
Vs6:  -0.0197142  0.00264894 -0.00432964
Vs7:  -0.0126433 -0.00386416 -0.00336789
Vs8:  -0.0095905 -0.00254645  -0.0124267
Vs9: -0.0186025 -0.0013351 -0.0233946
Vs10:  -0.0278384  0.00678363 -0.00748465





## structvio

Bgs0: 0.03100 0.00509 0.00592
Bgs1: 0.03100 0.00509 0.00592
Bgs2: 0.03100 0.00509 0.00592
Bgs3: 0.03100 0.00509 0.00592
Bgs4: 0.03100 0.00509 0.00592
Bgs5: 0.03100 0.00509 0.00592
Bgs6: 0.03100 0.00509 0.00592
Bgs7: 0.03100 0.00509 0.00592
Bgs8: 0.03100 0.00509 0.00592
Bgs9: 0.03100 0.00509 0.00592
Bgs10: 0.03100 0.00509 0.00592

_________________

## 调试记录

+ 限制了优化时间以及最大迭代次数，动态线特征权重，始终不固定Rws rmse 0.249
+ 等待稳定后固定Rws，跑飞了
+ 不限制优化时间，误差反倒变大，rmse 0.4+
+  





____________

