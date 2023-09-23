# Readme

```shell
ros2 launch cool_robot_bringup cool_robot.launch.py 

ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x00}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x80}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x00}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x06}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x07}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x0f}" -1

```

故障復位 
0x00 -> 0x80 -> 0x00

使驅動器的狀態機進入準備狀態servo on 
0x06 -> 0x07 -> 0x0f