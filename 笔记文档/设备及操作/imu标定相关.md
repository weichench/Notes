# **200hz**

## acc



![1566633284598](/home/slam/.config/Typora/typora-user-images/1566633284598.png)

加速度 noise 0.0006762



![1566633357793](/home/slam/.config/Typora/typora-user-images/1566633357793.png)

bias 0.0003039







## 陀螺仪

![1566633504361](/home/slam/.config/Typora/typora-user-images/1566633504361.png)

noise 

![1566633554703](/home/slam/.config/Typora/typora-user-images/1566633554703.png)

yaml文件中，均值/sprt（200）



acc noise 0.00073745

gyr noise 0.00005845



>> SCRIPT_allan_matparallel
>> opening the mat file.
>> loading timeseries.
>> imu frequency of 200.00.
>> sample period of 0.00500.
>> calculating allan deviation.
>> 时间已过 730.781759 秒。
>> saving to: results_20190825T092039.mat
>> done saving!
>> SCRIPT_noise_conversion


=================================
rostopic: /imu0
update_rate: 200.00
#Accelerometers
accelerometer_noise_density: 0.00059.
accelerometer_random_walk: 0.00000.
#Gyroscopes
gyroscope_noise_density: 0.000175.

gyroscope_random_walk: 0.000000.

>> SCRIPT_process_results
>> => opening the mat file.
>> => plotting accelerometer.
>> tau = 1.00 | tauid1 = 1094
>> h_fit1 slope = -0.5000 | y-intercept = -7.2335
>> h_fit2 slope = 0.5000 | y-intercept = -10.1378
>> tau = 3.01 | tauid2 = 1207
>> => plotting gyroscope.
>> tau = 1.00 | tauid1 = 1094
>> h_fit1 slope = -0.5000 | y-intercept = -9.7732
>> h_fit2 slope = 0.5000 | y-intercept = -12.6462
>> tau = 3.01 | tauid2 = 1207
>> => final results
>> accelerometer_noise_density = 0.00072019
>> accelerometer_random_walk   = 0.00006862
>> gyroscope_noise_density     = 0.00005682
>> gyroscope_random_walk       = 0.00000559



##  仿真程序运行说明的问题

### 问题主要在旋转（陀螺于相对旋转矩阵）



1. 陀螺仪的方差只要不差一个数量级，轨迹基本好
2. 加速度方差，即使差一个数量级，影响也不明显
3. 不提供初值，源程序直接估计Tic，效果极差，轨迹完全乱掉
4. 相机相对IMU的平移，即使误差差一个数量级，影响也不会过大，即使不优化该平移



# 相机存在的问题

两幅图像，原始图像，矫正后的图像，标定的结果不同, 

rect _的图像来自于sdk（相机内）的软件矫正后的图像，重新修改？

双目，立体矫正的，旋转图像，相当于再加一个旋转矩阵

### 即  原始图像标定得到的Tic，并不能直接用于矫正后图像的vio，应再乘上那个微小的旋转矩阵



Create a ROS bag containing the raw imag

在标定前, 注意测量格子的尺寸信息填入yaml文件







```
kalibr cam_rect
---

rosrun topic_tools throttle messages /mynteye/left/image_raw 4.0 /left

rosrun topic_tools throttle messages /mynteye/right/image_raw 4.0 /right



rosbag record -O stereo_calibra.bag /left /right
rosbag record -O stereo_calibra.bag /mynteye/left/image_raw  /mynteye/right/image_raw

source ros_ws/kalibr/devel/setup.bash



标相机
kalibr_calibrate_cameras --bag /home/slam/kalibr_workspace/src/kalibr-master/config/stereo_calibra.bag --topics /left /right --models pinhole-radtan pinhole-radtan --target /home/slam/kalibr_workspace/src/kalibr-master/config/april_6x6_80x80cm.yaml

kalibr_calibrate_cameras --bag /home/slam/kalibr_workspace/src/kalibr-master/config/stereo_calibra.bag --topics /mynteye/left/image_raw /mynteye/right/image_raw --models pinhole-radtan pinhole-radtan --target /home/slam/kalibr_workspace/src/kalibr-master/config/april_6x6_80x80cm.yaml

kalibr_calibrate_cameras --bag /home/slam/kalibr_workspace/src/kalibr-master/config/stereo_calibra.bag --topics /mynteye/left/image_raw /mynteye/right/image_raw --models pinhole-equi pinhole-equi --target /home/slam/kalibr_workspace/src/kalibr-master/config/april_6x6_80x80cm.yaml


kalibr_calibrate_cameras --bag /home/slam/kalibr_workspace/src/kalibr-master/config/stereo_calibra.bag --topics /mynteye/left/image_raw /mynteye/right/image_raw --models omni-radtan omni-radtan --target /home/slam/kalibr_workspace/src/kalibr-master/config/april_6x6_80x80cm.yaml


验证标定结果
kalibr_camera_validator --cam /home/slam/kalibr_workspace/src/kalibr-master/config/cam_chain.yaml --target /home/slam/kalibr_workspace/src/kalibr-master/config/april_6x6_80x80cm.yaml



记录相机-imu数据
rosbag record -O stereo_imu_calibra.bag /mynteye/left/image_raw /mynteye/right/image_raw /mynteye/imu/data_raw


联合标定
kalibr_calibrate_imu_camera --target /home/slam/kalibr_workspace/src/kalibr-master/config/april_6x6_80x80cm.yaml --cam /home/slam/kalibr_workspace/src/kalibr-master/config/cam_chain.yaml --imu /home/slam/kalibr_workspace/src/kalibr-master/imu.yaml --bag /home/slam/kalibr_workspace/src/kalibr-master/config/stereo_imu_calibra.bag --bag-from-to 5 40 --imu-models scale-misalignment \


kalibr_calibrate_imu_camera \
--target april_6x6_80x80cm.yaml \
--cam cam_chain.yaml \
--imu imu.yaml \
--bag stereo_imu_calibra.bag \
--bag-from-to 5 40 

    --timeoffset-padding 0.1


```





相机imu联合标定时的终端输出

```c
slam@slam-Default-string:~/kalibr_workspace/src/kalibr-master/config$ kalibr_calibrate_imu_camera \
> --target april_6x6_80x80cm.yaml \
> --cam cam_chain.yaml \
> --imu imu.yaml \
> --bag stereo_imu_calibra.bag \
> --bag-from-to 5 40 
importing libraries
Initializing IMUs:
  Update rate: 200.0
  Accelerometer:
    Noise density: 0.00072019 
    Noise density (discrete): 0.0101850246549 
    Random walk: 6.862e-05
  Gyroscope:
    Noise density: 5.682e-05
    Noise density (discrete): 0.00080355614614 
    Random walk: 5.59e-06
Initializing imu rosbag dataset reader:
	Dataset:          stereo_imu_calibra.bag
	Topic:            /mynteye/imu/data_raw
bagstart 1566808308.0
baglength 53.5450599194
[ WARN] [1566811039.856810]: BagImuDatasetReader: truncated 3700 / 10680 messages.
	Number of messages: 10680
Reading IMU data (/mynteye/imu/data_raw)
  Read 6980 imu readings over 35.0 seconds                   
Initializing calibration target:
  Type: aprilgrid
  Tags: 
    Rows: 6
    Cols: 6
    Size: 0.088 [m]
    Spacing 0.0264 [m]
Initializing camera chain:
Camera chain - cam0:
  Camera model: pinhole
  Focal length: [367.4200667650711, 367.0279118922714]
  Principal point: [388.55071122772165, 241.51954214437734]
  Distortion model: radtan
  Distortion coefficients: [-0.28174164925793577, 0.053456443982668765, 0.0005399025188997373, -0.0015697340701016718]
  baseline: no data available
Camera chain - cam1:
  Camera model: pinhole
  Focal length: [367.57960275687765, 367.2863954567346]
  Principal point: [369.33392560208307, 237.5342910084032]
  Distortion model: radtan
  Distortion coefficients: [-0.2959033498230458, 0.07057628151566844, 0.00032459655861134356, -0.0010156516967069168]
  baseline: [[ 0.99999319  0.00092173  0.00357248 -0.12015305]
 [-0.00092578  0.99999893  0.00113307  0.0002211 ]
 [-0.00357143 -0.00113637  0.99999298  0.00056274]
 [ 0.          0.          0.          1.        ]]
Initializing camera rosbag dataset reader:
	Dataset:          stereo_imu_calibra.bag
	Topic:            /mynteye/left/image_raw
[ WARN] [1566811040.751535]: BagImageDatasetReader: truncated 371 / 1071 images.
	Number of images: 1071
Extracting calibration target corners
  Extracted corners for 500 images (of 700 images)                              
Initializing camera rosbag dataset reader:
	Dataset:          stereo_imu_calibra.bag
	Topic:            /mynteye/right/image_raw
[ WARN] [1566811053.507682]: BagImageDatasetReader: truncated 371 / 1071 images.
	Number of images: 1071
Extracting calibration target corners
  Extracted corners for 505 images (of 700 images)                              
Baseline between cam0 and cam1 set to:
T=  [[ 0.99999319  0.00092173  0.00357248 -0.12015305]
 [-0.00092578  0.99999893  0.00113307  0.0002211 ]
 [-0.00357143 -0.00113637  0.99999298  0.00056274]
 [ 0.          0.          0.          1.        ]]
Baseline:  0.120154575396  [m]

Building the problem
	Spline order: 6
	Pose knots per second: 100
	Do pose motion regularization: False
		xddot translation variance: 1000000.000000
		xddot rotation variance: 100000.000000
	Bias knots per second: 50
	Do bias motion regularization: True
	Blake-Zisserman on reprojection errors -1
	Acceleration Huber width (sigma): -1.000000
	Gyroscope Huber width (sigma): -1.000000
	Do time calibration: True
	Max iterations: 30
	Time offset padding: 0.030000
Estimating time shift camera to imu:

Initializing a pose spline with 3492 knots (100.000000 knots per second over 34.922630 seconds)
  Time shift camera to imu (t_imu = t_cam + shift):
0.0
Estimating time shift camera to imu:

Initializing a pose spline with 3492 knots (100.000000 knots per second over 34.922630 seconds)
  Time shift camera to imu (t_imu = t_cam + shift):
0.0

Estimating imu-camera rotation prior

Initializing a pose spline with 3492 knots (100.000000 knots per second over 34.922630 seconds)
Gravity was intialized to [ 0.06808498 -9.79931903  0.37031586] [m/s^2]
  Orientation prior camera-imu found as: (T_i_c)
[[-0.00274824  0.99968341 -0.02501069]
 [-0.99959725 -0.00203985  0.02830508]
 [ 0.0282451   0.02507841  0.99928639]]
  Gyro bias prior found as: (b_gyro)
[ 0.00537146  0.00288141 -0.00072736]

Initializing a pose spline with 3504 knots (100.000000 knots per second over 35.042630 seconds)

Initializing the bias splines with 1752 knots

Adding camera error terms (/mynteye/left/image_raw)
  Added 500 camera error terms                                

Adding camera error terms (/mynteye/right/image_raw)
  Added 505 camera error terms                                

Adding accelerometer error terms (/mynteye/imu/data_raw)
  Added 6977 of 6980 accelerometer error terms (skipped 3 out-of-bounds measurements)

Adding gyroscope error terms (/mynteye/imu/data_raw)
  Added 6977 of 6980 gyroscope error terms (skipped 3 out-of-bounds measurements)

Before Optimization
===================
Normalized Residuals
----------------------------
Reprojection error (cam0):     mean 2.35837325843, median 1.96301949391, std: 1.75738252864
Reprojection error (cam1):     mean 3.30153303272, median 2.1465080769, std: 7.10668917037
Gyroscope error (imu0):        mean 570.177346258, median 396.026360463, std: 537.847212463
Accelerometer error (imu0):    mean 353.964059748, median 178.144727353, std: 457.050249405

Residuals
----------------------------
Reprojection error (cam0) [px]:     mean 2.35837325843, median 1.96301949391, std: 1.75738252864
Reprojection error (cam1) [px]:     mean 3.30153303272, median 2.1465080769, std: 7.10668917037
Gyroscope error (imu0) [rad/s]:     mean 0.458169510976, median 0.318229415984, std: 0.432190433259
Accelerometer error (imu0) [m/s^2]: mean 3.60513267546, median 1.81440844023, std: 4.6550680587

Optimizing...
Using the block_cholesky linear system solver
Using the levenberg_marquardt trust region policy
Using the block_cholesky linear system solver
Using the levenberg_marquardt trust region policy
Initializing
Optimization problem initialized with 7028 design variables and 127780 error terms
The Jacobian matrix is 269512 x 31606
[0.0]: J: 6.62217e+09
[1]: J: 1.48432e+08, dJ: 6.47373e+09, deltaX: 0.466825, LM - lambda:10 mu:2
[2]: J: 2.08354e+06, dJ: 1.46349e+08, deltaX: 2.05864, LM - lambda:3.33333 mu:2
[3]: J: 1.69307e+06, dJ: 390466, deltaX: 9.04695, LM - lambda:1.11111 mu:2
[4]: J: 1.69215e+06, dJ: 925.102, deltaX: 24.5271, LM - lambda:0.37037 mu:2
[5]: J: 1.6919e+06, dJ: 251.819, deltaX: 15.5093, LM - lambda:0.370108 mu:2
[6]: J: 1.69168e+06, dJ: 215.883, deltaX: 17.9619, LM - lambda:0.370047 mu:2
[7]: J: 1.69145e+06, dJ: 229.874, deltaX: 14.7306, LM - lambda:0.370082 mu:2
[8]: J: 1.69117e+06, dJ: 281.611, deltaX: 18.5946, LM - lambda:0.3653 mu:2
[9]: J: 1.69081e+06, dJ: 359.173, deltaX: 17.6971, LM - lambda:0.364574 mu:2
[10]: J: 1.69032e+06, dJ: 489.754, deltaX: 22.6814, LM - lambda:0.352939 mu:2
[11]: J: 1.68966e+06, dJ: 663.583, deltaX: 23.5215, LM - lambda:0.348131 mu:2
[12]: J: 1.68881e+06, dJ: 847.58, deltaX: 28.9619, LM - lambda:0.329872 mu:2
[13]: J: 1.68795e+06, dJ: 863.857, deltaX: 27.6547, LM - lambda:0.323631 mu:2
[14]: J: 1.68727e+06, dJ: 679.044, deltaX: 26.1189, LM - lambda:0.318697 mu:2
[15]: J: 1.68679e+06, dJ: 475.958, deltaX: 24.686, LM - lambda:0.318303 mu:2
[16]: J: 1.68646e+06, dJ: 331.88, deltaX: 20.4833, LM - lambda:0.318261 mu:2
[17]: J: 1.68622e+06, dJ: 238.065, deltaX: 17.6228, LM - lambda:0.318261 mu:2
[18]: J: 1.68605e+06, dJ: 176.283, deltaX: 15.116, LM - lambda:0.318261 mu:2
[19]: J: 1.68591e+06, dJ: 134.587, deltaX: 13.5068, LM - lambda:0.318261 mu:2
[20]: J: 1.68581e+06, dJ: 105.004, deltaX: 12.2585, LM - lambda:0.318261 mu:2
[21]: J: 1.68572e+06, dJ: 83.3643, deltaX: 11.0388, LM - lambda:0.318262 mu:2
[22]: J: 1.68566e+06, dJ: 66.9713, deltaX: 10.0049, LM - lambda:0.318263 mu:2
[23]: J: 1.6856e+06, dJ: 54.3031, deltaX: 9.05827, LM - lambda:0.318264 mu:2
[24]: J: 1.68556e+06, dJ: 44.3252, deltaX: 8.21779, LM - lambda:0.318264 mu:2
[25]: J: 1.68552e+06, dJ: 36.3588, deltaX: 7.46506, LM - lambda:0.318265 mu:2
[26]: J: 1.68549e+06, dJ: 29.9362, deltaX: 6.7858, LM - lambda:0.318266 mu:2
[27]: J: 1.68547e+06, dJ: 24.7184, deltaX: 6.17374, LM - lambda:0.318266 mu:2
[28]: J: 1.68545e+06, dJ: 20.4562, deltaX: 5.62151, LM - lambda:0.318267 mu:2
[29]: J: 1.68543e+06, dJ: 16.958, deltaX: 5.12382, LM - lambda:0.318268 mu:2
[30]: J: 1.68541e+06, dJ: 14.0795, deltaX: 4.66933, LM - lambda:0.318268 mu:2

After Optimization (Results)
==================
Normalized Residuals
----------------------------
Reprojection error (cam0):     mean 1.84720078743, median 0.803894758001, std: 3.02238618474
Reprojection error (cam1):     mean 1.36376795759, median 0.737857875255, std: 3.56224812522
Gyroscope error (imu0):        mean 1.73079284821, median 1.35942919783, std: 1.32508608569
Accelerometer error (imu0):    mean 3.02830107554, median 2.39160739958, std: 2.51583099561

Residuals
----------------------------
Reprojection error (cam0) [px]:     mean 1.84720078743, median 0.803894758001, std: 3.02238618474
Reprojection error (cam1) [px]:     mean 1.36376795759, median 0.737857875255, std: 3.56224812522
Gyroscope error (imu0) [rad/s]:     mean 0.00139078923088, median 0.00109237768716, std: 0.00106478106832
Accelerometer error (imu0) [m/s^2]: mean 0.0308433211167, median 0.0243585803294, std: 0.0256238007177

Transformation T_cam0_imu0 (imu0 to cam0, T_ci): 
[[-0.00288919  0.99971588 -0.02366034  0.06129036]
 [-0.99990859 -0.00320065 -0.01313638  0.0178013 ]
 [-0.01320837  0.02362022  0.99963374 -0.01902964]
 [ 0.          0.          0.          1.        ]]

cam0 to imu0 time: [s] (t_imu = t_cam + shift)
0.00700167229434

Transformation T_cam1_imu0 (imu0 to cam1, T_ci): 
[[-0.003858    0.99979051 -0.02010111 -0.05891469]
 [-0.99991981 -0.0040994  -0.01198181  0.01794408]
 [-0.0120617   0.02005328  0.99972615 -0.01870589]
 [ 0.          0.          0.          1.        ]]

cam1 to imu0 time: [s] (t_imu = t_cam + shift)
0.00724075282503

IMU0:
----------------------------
  Model: calibrated
  Update rate: 200.0
  Accelerometer:
    Noise density: 0.00072019 
    Noise density (discrete): 0.0101850246549 
    Random walk: 6.862e-05
  Gyroscope:
    Noise density: 5.682e-05
    Noise density (discrete): 0.00080355614614 
    Random walk: 5.59e-06
  T_i_b
    [[ 1.  0.  0.  0.]
     [ 0.  1.  0.  0.]
     [ 0.  0.  1.  0.]
     [ 0.  0.  0.  1.]]
  time offset with respect to IMU0: 0.0 [s]

  Saving camera chain calibration to file: camchain-imucam-stereo_imu_calibra.yaml

  Saving imu calibration to file: imu-stereo_imu_calibra.yaml
  Detailed results written to file: results-imucam-stereo_imu_calibra.txt
Generating result report...
/home/slam/kalibr_workspace/src/kalibr-master/Schweizer-Messer/sm_python/python/sm/PlotCollection.py:57: wxPyDeprecationWarning: Using deprecated class PySimpleApp. 
  app = wx.PySimpleApp()
  Report written to report-imucam-stereo_imu_calibra.pdf

```



body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ -0.00288919, -0.99990859, -0.01320837,  0.0177254, 
  0.99971588, -0.00320065,  0.02362022, -0.06076648,
 -0.02366034, -0.01313638,  0.99963374,  0.02070666,
  0.,         0.,          0.,          1.        ]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ -0.003858,   -0.99991981, -0.0120617,   0.01748973,
  0.99979051, -0.0040994,   0.02005328,  0.05935102,
 -0.02010111, -0.01198181,  0.99972615,  0.01773152,
 0.,          0. ,         0.,          1.        ]