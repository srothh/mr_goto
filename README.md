# mr_goto
Launch Map:\
ros2 launch stage_ros2 stage.launch.py

Start GoTo:\
ros2 run mr_goto goto --ros-args -p mode:="plan" -p x:=-5.0 -p y:=-6.0 --remap scan:=base_scan
