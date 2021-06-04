### 改变话题发布的频率

```
rosrun topic_tools throttle messages /mynteye/left/image_raw 4.0 /left
```

```
rosbag record -O stereo_calibra.bag /left /right
-o  表示制定了文件名

rosbag play /home/slam/data_set/V1_02_medium.bag 


rosbag record -O structural_line_12.bag /linefeature_tracker/linefeature_1  /linefeature_tracker/linefeature_2  /linefeature_tracker/linefeature_3

rosbag record -O structural_line_data2.bag /linefeature_tracker/linefeature_1  /linefeature_tracker/linefeature_2  /linefeature_tracker/linefeature_3



rosbag record -O structural_line_white_car.bag /linefeature_tracker/linefeature_1  /linefeature_tracker/linefeature_2  /linefeature_tracker/linefeature_3


rosbag record -O line1.bag /linefeature_tracker/linefeature_1  

tar -xvf  解压tar文件
```

## 有关launch文件

+ param参数，程序可读              arag参数，仅ros识别



### 新建ros工作空间

```c++
mkdir -p ~/kalibr_workspace/src
 cd ~/kalibr_workspace 
 cd src
 catkin_init_workspace
Creating symlink "/home/slam/kalibr_workspace/src/CMakeLists.txt" pointing to "/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake"
 cd .. 
 catkin_make
  
  
slam@slam-Default-string:~/kalibr_workspace$ echo "source  ～/kalibr_workspace/devel/setup.bash">>~/.bashrc
slam@slam-Default-string:~$ gedit .bashrc

source ~/.bashrc    更新指令
  当前终端有效
   source ~/catkin_ws_m/devel/setup.bash

ROS的空间可以建立多个，只需要将环境变量添加即可：     
echo "source /home/slam/catkin_ws_1/devel/setup.bash">>~/.bashrc 


```



##  小觅相机的话题

```c
/mynteye/camera_mesh
/mynteye/depth/camera_info
/mynteye/depth/image_raw
/mynteye/depth/image_raw/compressed
/mynteye/depth/image_raw/compressed/parameter_descriptions
/mynteye/depth/image_raw/compressed/parameter_updates
/mynteye/depth/image_raw/theora
/mynteye/depth/image_raw/theora/parameter_descriptions
/mynteye/depth/image_raw/theora/parameter_updates
/mynteye/disparity/camera_info
/mynteye/disparity/image_norm
/mynteye/disparity/image_norm/compressed
/mynteye/disparity/image_norm/compressed/parameter_descriptions
/mynteye/disparity/image_norm/compressed/parameter_updates
/mynteye/disparity/image_norm/theora
/mynteye/disparity/image_norm/theora/parameter_descriptions
/mynteye/disparity/image_norm/theora/parameter_updates
/mynteye/disparity/image_raw
/mynteye/disparity/image_raw/compressed
/mynteye/disparity/image_raw/compressed/parameter_descriptions
/mynteye/disparity/image_raw/compressed/parameter_updates
/mynteye/disparity/image_raw/theora
/mynteye/disparity/image_raw/theora/parameter_descriptions
/mynteye/disparity/image_raw/theora/parameter_updates
/mynteye/imu/data_raw
/mynteye/left/camera_info
/mynteye/left/image_raw
/mynteye/left/image_raw/compressed
/mynteye/left/image_raw/compressed/parameter_descriptions
/mynteye/left/image_raw/compressed/parameter_updates
/mynteye/left/image_raw/theora
/mynteye/left/image_raw/theora/parameter_descriptions
/mynteye/left/image_raw/theora/parameter_updates
/mynteye/left_rect/camera_info
/mynteye/left_rect/image_rect
/mynteye/left_rect/image_rect/compressed
/mynteye/left_rect/image_rect/compressed/parameter_descriptions
/mynteye/left_rect/image_rect/compressed/parameter_updates
/mynteye/left_rect/image_rect/theora
/mynteye/left_rect/image_rect/theora/parameter_descriptions
/mynteye/left_rect/image_rect/theora/parameter_updates
/mynteye/points/data_raw
/mynteye/right/camera_info
/mynteye/right/image_raw
/mynteye/right/image_raw/compressed
/mynteye/right/image_raw/compressed/parameter_descriptions
/mynteye/right/image_raw/compressed/parameter_updates
/mynteye/right/image_raw/theora
/mynteye/right/image_raw/theora/parameter_descriptions
/mynteye/right/image_raw/theora/parameter_updates
/mynteye/right_rect/camera_info
/mynteye/right_rect/image_rect
/mynteye/right_rect/image_rect/compressed
/mynteye/right_rect/image_rect/compressed/parameter_descriptions
/mynteye/right_rect/image_rect/compressed/parameter_updates
/mynteye/right_rect/image_rect/theora
/mynteye/right_rect/image_rect/theora/parameter_descriptions
/mynteye/right_rect/image_rect/theora/parameter_updates
/mynteye/temperature/data_raw
/rosout
/rosout_agg

```



缺少相关依赖项：

```c
import igraph
ImportError: No module named igraph


可搜索   sudo apt-get install    igraph
得：
sudo apt-get install python2.7-igraph
    sudo apt-get -f install  修复安装问题
```

##### 查看cpu温度

psensor

cat /proc/cpuinfo |grep MHz|uniq

_________



## 释放swap

sync                  sudo swapoff -a                       sudo swapon -a

