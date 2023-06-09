# mr_goto
Launch Map:\
ros2 launch stage_ros2 stage.launch.py

Start GoTo:\
ros2 run mr_goto goto --ros-args -p mode:="plan" -p x:=-3.0 -p y:=-5.0 -p deg:=70.0 --remap scan:=base_scan

Call explored Area Service:\
ros2 service call /explored_area std_srvs/srv/Trigger