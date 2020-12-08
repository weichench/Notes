```

roslaunch vins_estimator tum.launch
roslaunch vins_estimator vins_rviz.launch

evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum -as -va --plot 

evo_ape tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum /home/slam/catkin_ws_2/src/git_project/PL-VIO-master/config/traj/vins_result.tum  -as -va --plot 

/home/slam/Dataset/MH_03_medium/mav0/state_groundtruth_estimate0

evo_ape tum /home/slam/Dataset/MH_03_medium/mav0/state_groundtruth_estimate0/data_.tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum -as  -va --plot 

evo_ape tum /home/slam/Dataset/MH_03_medium/mav0/state_groundtruth_estimate0/data.tum ~/catkin_ws_4/src/git_project/VINS-Fusion/vio.tum -as  -va --plot 

evo_traj tum /home/slam/catkin_ws_6/src/VINS-Mono-master/vio.tum  --plot
evo_traj tum /home/slam/Dataset/Tum/dataset-corridor4_512_16/mav0/mocap0/data.tum   --plot

```

