# Laser Object Extraction in Python

This is a ROS package, where I have implemented Laser Object Extraction using Python and ROS.

To run files, simply add this package to your catkin workspace and run ```$ catkin_make```

<!-- ## 

### Background
The aim of this task was to create an algorithm that can reduce the total number of path points for robot navigation while preserving the main shape to follow. For this task, the main assumption is that the approximate total number of path points is already given and is equal to ~992. As the input of the path is not given as a list of points and rather sent one by one, it was decided to update path points on-line and publish them on other topic.

### Running
To run the example, simply run
```bash
roslaunch laser_object_extractor path_reduction_launch.launch
```

The script will run the rosbag file, execution node, and will open RViZ window. To change the desired number of points, simply change it in ```<param name="desired_path_num" type="int" value="100" />``` line.

![](/assets/path.png)

### Solution video
Online path data reduction
https://youtu.be/lBLjt2U8U7I -->

## Rack Legs Extraction in warehouse - usage example

### Background
The aim of this example is to find and locate the legs of the rack which are installed in the warehouse. The task is to locate and track rack legs in the warehouse, where the robot is moving. The data is given in the recorded rosbag and consists only of LaserScan and Image sensor readings. 

![](/assets/input.png)

### Solution
The solution to this task is partially inspired by `Przybyła, M. (2017, July). Detection and tracking of 2D geometric obstacles from LRF data. In 2017 11th International Workshop on Robot Motion and Control (RoMoCo) (pp. 135-141). IEEE.` work, where the author has presented an idea for LaserScan segmentation based on adjacent point distances. The points in the LaserScan are traversed through each angle, and it's value, range, and distance values are used for computation of their affiliation to any LaserScan segments. These segments are called PointSets and are shown below:

![](/assets/segments.png)

Afterward, every segment is checked for multiple criteria, and the ones that satisfy all, can be assumed to be rack legs candidates as shown below:

![](/assets/rack_points_new.png)

### Running
Please add a corresponding rosbag to the /rviz folder or change path to it.

To run the example, simply run
```bash
roslaunch laser_object_extractor rack_legs_extraction_launch.launch
```

The script will run the rosbag file, execution node, and will open RViZ window.

### Limitations and discussion
It can be clearly seen both from screenshots and video recordings that although rack legs are mostly identified correctly, there are some candidates that are not rack legs that robot is looking for. Therefore, there is some space for improvement.

Although only LaserScan readings were used, these rack locations can not be separately found using only RGB camera readings. Although rack legs can be clearly identified through segmentation or rack detection, camera due to its nature can not by itself give appropriate measure of their distance and location. 

Therefore, sensor fusion has to be used to improve detection accuracy. The main limitation of this implementation is that it is not using the camera's data for rack detection. For example, given the proper transformation relations between those two sensors(unfortunately, this information was not given), camera features like focal length, skew, distortions, etc., each laser point could be mapped to a corresponding pixel value. Afterward, the image could be divided into rack legs and else regions, which are then combined with calculated candidates. This could greatly reduce the number of false positives.

Finally, it can be seen that at some time instances not all rack legs are identified due to the occlusion, but they are still detected when robot displaces to other location. This is limitation of the laser sensor.
<!-- 
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE)
 -->

### Solution video 
Rack Leg extraction from LaserScan
https://youtu.be/YnFh-PRTJPY
