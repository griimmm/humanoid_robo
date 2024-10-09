# humanoid_robo
Project
To start the communication with Arduino
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 921600
```

To run the moveit setup 

```
ros2 launch hubert_grp demo.launch.py
```
To run the joint value publisher to arduino

```
ros2 run move_ee joint_pub
```

To run the moveit from program

```
ros2 run move_ee move_program
```

4 publishers, 6 subscribers allowed on the arduino after rebuilding the core library