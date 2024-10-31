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
ros2 run move_ee joint_publisher
```
To run trajectory planning and moveit communication

```
ros2 launch cpp_move_goal move_hubert.launch.py
```
To run the CV coordinate publisher

```
ros2 run move_ee coord_publisher
```

Arduino re-compiled library:

4 publishers, 6 subscribers allowed on the arduino after rebuilding the core library


Future Scope:

Improvement of IK Plugin