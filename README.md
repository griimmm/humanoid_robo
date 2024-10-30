# humanoid_robo
Project
To start the communication with Arduino
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

To run the moveit setup with rviz 

```
ros2 launch hubert_group_moveit demo.launch.py
```
To run the joint value publisher to arduino

```
ros2 run move_ee joint_pub
```

To run the moveit from program

```
ros2 launch move_ee try.launch.py
```
To run the CV coord publisher

```
ros2 run move_ee coord_publisher
```

Arduino re-compiled library:

4 publishers, 6 subscribers allowed on the arduino after rebuilding the core library


Future Scope:

Improvement of IK Plugin