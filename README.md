# z1_nmpc_pkg
ocs2 sqp mpc implementation on unitree z1

准备：替换ocs2_mobile_manipulator_ros 
     将模型数据文档unitree_z1添加到ocs2_robotic_assets/resources/mobile_manipulator/文件夹下
     在ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/文件夹中新建unitree_z1文件夹，并将配置文件task.info添加进去

2 运行 rosrun unitree_z1_mpc policy_to_real
3 运行 ./z1_ctrl
4 运行 roslaunch ocs2_mobile_manipulator_ros z1_real_arm.launch
