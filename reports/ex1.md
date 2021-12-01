# LINK TO REPORT:

https://www.overleaf.com/1146151537xfwzbkpxshrb











# Exercise 1: Exploration and Mapping

In this assignment, we were challenged with programming the robot to create a map of the CS lower ground floor.

## Our approach
We broke the problem down into three steps:

1. Move the robot around the floor to explore the area.

2. Record all laser data while doing so.

3. Translate all laser data into a readable map.

### Initial attempt
The first step was initially accomplished using the joy-motion controller program, allowing us to drive the robot around using a joystick.

This was executed with the Bash commands:

```
roscore
roslaunch socspioneer p2os_laser.launch
roslaunch socspioneer teleop_joy.launch
```

To record the laser data a ROS 'bag' has to be used to hold all the messages from the robot. This is created with the command:

```
rosbag record -O laser.bag /base_scan /tf /odom
```

We then drove the robot around, doing 2 laps of the lower floor to ensure we did not miss any areas.
This generated a 35MB file. In order to create the map we then ran the commands:

```
rosparam set use_sime_time true
rosrun gmapping slam_gmapping scan:=base_scan
rosbag play laser.bag
```

This then replays the recorded data into GMapping which generates a greyscale bitmap of the laser scans.

***INSERT IMAGE OF MAP HERE***

An issue we encountered here was that the lasers picked up a non-existent area, seen here near the top of the map. We believe that they were reflected from the glass wall to the right of the robot lab, making it appear as if there is an additional area. ***(say how we solved this if we manage to fix it, else say how we might fix it in future. Sonar maybe?)***

### Automating the motion
While our initial approach completed the task and we were able to produce a map, we wanted to extend this so that the robot could explore without us having to control it. To do this, we adapted the [sample wall hugger code](http://www.cs.bham.ac.uk/internal/courses/int-robot/2018/notes/wallhugger.php) from Python to C++.

Given the width  and geometry of the lower ground floor, we believed this would be a good solution since the robot would be able to follow the walls and would, hopefully, not miss any areas in the middle of the room.
***add more detail here***

***what issues did we encounter with getting this working? How did we solve them?***

***what is left to be desired in the map produced by this method? possible extensions?***

