# color_sensor_pid

PID Turtlebot3 lane following projet using a Flora color sensor

# Notice

The node subscribes to error topic of the calculate error node. This value is used to compute the robot command in order to follow the lane.

### Subscribed Topics

* ```/Error``` ([std_msgs/Float32])
    Error between the luminous intensity measured and the one targeted
    
### Published Topics

* ``` /cmd_vel``` ([geometry_msgs/Twist])
    Publishes the computed linear and angular speed of the robot

# How to build
```
cd ~/catkin_ws/src/
git clone https://github.com/Thermay-Robotics/color_sensor_pid.git
cd ~/catkin_ws
catkin_make
```

# Run

```
roslaunch color_sensor_pid pid_color_sensor.launch
```
