# Readme

```shell
ros2 launch cool_robot_bringup cool_robot.launch.py 
```

**透過topic 一步步控制control_word 來 servo on**
```shell
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x00}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x80}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x00}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x06}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x07}" -1
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x0f}" -1
```
servo off
```shell
ros2 topic pub /control_word std_msgs/msg/UInt16 "{data: 0x00}" -1
```


**透過call service的方式servo on**

```shell
ros2 service call /cool_robot_controller/servo std_srvs/srv/SetBool "data: True" 
```
servo off
```shell
ros2 service call /cool_robot_controller/servo std_srvs/srv/SetBool "data: False"
```

>故障復位 
>0x00 -> 0x80 -> 0x00
>
>使驅動器的狀態機進入準備狀態servo on 
>0x06 -> 0x07 -> 0x0f