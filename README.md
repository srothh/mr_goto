# mr_goto
Launch Map:\
ros2 launch stage_ros2 stage.launch.py

Start GoTo:\
ros2 run mr_goto goto --ros-args -p mode:="plan" -p x:=-3.0 -p y:=-5.0 -p deg:=70.0 --remap scan:=base_scan --enclave /ws02/src/mr_goto

Call explored Area Service:\
ros2 service call /explored_area std_srvs/srv/Trigger


create DDS Security:\
ros2 security create_keystore sros_keystore

add the following code to your env.sh file:\
export ROS_SECURITY_KEYSTORE=~/projects/mobile_robotics/sros_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
echo "activated ROS_SECURITY" 

in your Makefile change under build-ws01 and build-ws02:\
the line with colon to:\
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DSECURITY=ON

execute follwing line in your terminals to creat key for every node:\
ros2 security create_enclave sros_keystore /ws01/src/stage_ros2
ros2 security create_enclave sros_keystore /ws02/src/mr_ekf
ros2 security create_enclave sros_keystore /ws02/src/mr_goto
ros2 security create_enclave sros_keystore /ws02/src/mr_move
ros2 security create_enclave sros_keystore /ws02/src/mr_nav2
ros2 security create_enclave sros_keystore /ws02/src/mr_pf
ros2 security create_enclave sros_keystore /ws02/src/mr_viz

to see the security stream:\
sudo tcpdump -X -i any udp port 7400

