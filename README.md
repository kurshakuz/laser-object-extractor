# Movel.AI Technical Assignment

This is a ROS package, where I have implemented given assignment tasks.

To run files, simply add this package to your catkin workspace and run ```$ catkin_make```

## Part A - Algorithmic Test

### Background
The aim of this task was to create an algorithm which can reduce total number of path points for robot navigation while preserving main shape to follow. For this task, the main assumption is that approximate total number of path points is already given and is equal to ~992. As the input of path is not given as a list of points and rather sent one by one, it was decided to update path points on-line and publish them on other topic. 

### Running
To run the example, simply run
```bash
roslaunch movel_assignment path_reduction_launch.launch
```

The script will run the rosbag file, execution node, and will open RViZ window. To change the desired number of points, simply change it in ```<param name="desired_path_num" type="int" value="100" />``` line.

![](/assets/path.png)

### Solution video
Online path data reduction - Assignment, Part A:
https://youtu.be/lBLjt2U8U7I

## Part B - Rack Detection Test

### Background
The aim of this task was to find and locate legs of the rack which are installed in the warehouse. The task is to locate and track rack legs in the warehouse, where robot is moving. The data is given in the recorded rosbag and consists only of LaserScan and Image sensor readings. 

![](/assets/input.png)

### Solution
The solution of this task is partially inspired by `Przybyła, M. (2017, July). Detection and tracking of 2d geometric obstacles from lrf data. In 2017 11th International Workshop on Robot Motion and Control (RoMoCo) (pp. 135-141). IEEE.` work, where author have presented an idea for LaserScan segmentation based on adjacent point distances. The points in the LaserScan are traversed through each angle, and it's value, range and distance values are used for computation of their affiliation to any LaserScan segments. These segments are called PointSets and are shown below:

![](/assets/segments.png)

Afterwards, every segment is checked for multiple criterions, and the ones that satisfies all, can be assumed to be rack legs candidates as shown below:

![](/assets/rack_points_new.png)

### Running
Please add a corresponding rosbag to the /rviz folder or change path to it.

To run the example, simply run
```bash
roslaunch movel_assignment rack_legs_extraction_launch.launch
```

The script will run the rosbag file, execution node, and will open RViZ window.

### Discussion and ideas for improvement
It can be clearly seen both from screenshots and video recordings that although rack legs are mostly identified correctly, there are some candidates that are not rack legs that robot is looking for. Therefore, there is some space for improvement.

Although only LaserScan reading were used, these rack locations can not be separately found using only RGB camera readings. Although, rack legs can be clearly identified through segmentation or rack detection, camera due to its nature can not by itself give appropriate measure of their distance and location. 

Therefore, sensor fusion have to be used to improve detection accuracy. The main limitation of this implementation is that it is not using camera's data for rack detection. For example, given the proper transformation relations between those two sensors(unfortunately, this information was not given), camera's features like focal length, skew, distortions etc., each laser point could be mapped to a corresponding pixel value. Afterwards, image could be divided into rack legs and else regions, which are then combined with calculated candidates. This could greatly reduce number of false positives.

Finally, it can be seen that at some time instances not all rack legs are identified due to the occlusion, but they are still detected when robot displaces to other location. This is limitation of the laser sensor.
<!-- 
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE)
 -->

### Solution video 
Rack Leg extraction from LaserScan - Assignment, Part B: 
https://youtu.be/YnFh-PRTJPY
