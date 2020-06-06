#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PointSet:
  def __init__(self):
    self.num_points = 0
    self.is_visible = False
    self.start = -1
    self.end = -1

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

class Extractor:
    def __init__(self, laserscan_sub_topic, raw_obstacles_pub_topic, rack_legs_pub_topic):
        self.is_showing_circles = True
        self.avg_radius = 0.1
        self.p_min_group_points = 3
        self.p_max_group_points = 21
        self.p_max_group_distance = 0.1
        self.p_distance_proportion = 0.00628
        self.p_max_distance_between_points = 0.1
        self.p_max_distance_from_base = 5.0
        self.p_frame_id = "laser"
        self.input_points = []
        self.point_segments = []
        self.point_circles = []
        self.segment_counter = 0
        # Publisher for obstacles
        if (self.is_showing_circles):
            self.obstacles_pub = rospy.Publisher(rack_legs_pub_topic, MarkerArray, queue_size=10)
        else:
            self.obstacles_pub = rospy.Publisher(raw_obstacles_pub_topic, Marker, queue_size=10)

        rospy.Subscriber(laserscan_sub_topic, LaserScan, self.callback_segments)

    def process_points(self):
        self.point_segments = []
        self.point_circles = []

        self.groupPoints()

        if self.is_showing_circles:
            self.publishCircles()
        else:
            self.publishObstacles()

        self.input_points = []
        
    def groupPoints(self):
        sin_dp = math.sin(2.0 * self.p_distance_proportion)

        point_set = PointSet()
        point_set.start = 0
        point_set.end = 0
        point_set.num_points = 1
        point_set.is_visible = True

        for i, point in enumerate(self.input_points):
            rrange = np.linalg.norm(point)
            distance = np.linalg.norm((point[0] - self.input_points[point_set.end][0], point[1] - self.input_points[point_set.end][1]))

            if (distance < self.p_max_group_distance + rrange * self.p_distance_proportion):
                point_set.end = i
                point_set.num_points += 1
            else:
                prev_range = np.linalg.norm(self.input_points[point_set.end])

                # Heron's equation
                p = (rrange + prev_range + distance) / 2.0
                S = math.sqrt(p * (p - rrange) * (p - prev_range) * (p - distance))
                sin_d = 2.0 * S / (rrange * prev_range)

                if (abs(sin_d) < sin_dp and rrange < prev_range):
                    point_set.is_visible = False

                self.detectSegments(point_set)

                point_set.start = i
                point_set.end = i
                point_set.num_points = 1
                point_set.is_visible = (abs(sin_d) > sin_dp or rrange < prev_range)

        self.detectSegments(point_set)

    def detectSegments(self, point_set):
        if (point_set.num_points < self.p_min_group_points or \
                point_set.num_points > self.p_max_group_points or \
                    (not point_set.is_visible)):
            
            return

        p1 = Point()
        p1.x = self.input_points[point_set.start][0]
        p1.y = self.input_points[point_set.start][1]
            
        p2 = Point()
        p2.x = self.input_points[point_set.end][0]
        p2.y = self.input_points[point_set.end][1]

        distance_between_points = math.hypot(p2.x - p1.x, p2.y - p1.y)
        distance_from_center_p1 = math.hypot(p1.x, p1.y)
        distance_from_center_p2 = math.hypot(p2.x, p2.y)

        if (distance_between_points < self.p_max_distance_between_points) \
                and ((distance_from_center_p1 < self.p_max_distance_from_base) \
                    and (distance_from_center_p2 < self.p_max_distance_from_base)):

            if (self.is_showing_circles):
                x_accum = 0
                y_accum = 0
                for i in range(point_set.start, point_set.end):
                    x_accum += self.input_points[i][0]
                    y_accum += self.input_points[i][1]

                p = Point()
                num_elems =  point_set.end - point_set.start
                p.x = float(x_accum)/num_elems
                p.y = float(y_accum)/num_elems 

                self.point_circles.append(p)
            else:
                self.point_segments.append(p1)
                self.point_segments.append(p2)

    def publishObstacles(self):
        obstacles_msg = Marker()
        obstacles_msg.header.stamp = self.stamp
        obstacles_msg.header.frame_id = self.p_frame_id
        obstacles_msg.ns = "lines"
        obstacles_msg.id = 0

        obstacles_msg.type = Marker.LINE_LIST
        obstacles_msg.action = Marker.ADD
        obstacles_msg.scale.x = 0.1

        obstacles_msg.color.a = 1.0
        obstacles_msg.color.r = 0.0
        obstacles_msg.color.g = 1.0
        obstacles_msg.color.b = 0.0

        obstacles_msg.points = self.point_segments

        self.obstacles_pub.publish(obstacles_msg)

    def publishCleanCircles(self):
        rack_legs_msg = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.stamp
        marker.header.frame_id = self.p_frame_id
        marker.ns = "legs"

        marker.type = Marker.CYLINDER
        marker.action = Marker.DELETEALL

        rack_legs_msg.markers.append(marker)
        self.obstacles_pub.publish(rack_legs_msg)

    def publishCircles(self):
        self.publishCleanCircles()
        # print(self.point_circles)
        rack_legs_msg = MarkerArray()

        for i, point in enumerate(self.point_circles):
            marker = Marker()
            marker.header.stamp = self.stamp
            marker.header.frame_id = self.p_frame_id
            marker.ns = "legs"
            marker.id = i

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = self.avg_radius
            marker.scale.y = self.avg_radius
            marker.scale.z = 0.01

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = 0

            rack_legs_msg.markers.append(marker)

        self.obstacles_pub.publish(rack_legs_msg)

    def callback_segments(self, laserData):
        '''
        Main method of the Extractor node.
        Used by Extractor subscriber.
        
        Parameters: 
            laserData (LaserScan): data that is recieved by the subscriber.
        '''
        self.base_frame_id = laserData.header.frame_id
        self.stamp = laserData.header.stamp

        phi = laserData.angle_min

        for r in laserData.ranges:
            if (r >= laserData.range_min and r <= laserData.range_max):
                # self.input_points.append((r, phi))
                self.input_points.append(pol2cart(r, phi))

            phi += laserData.angle_increment

        self.process_points()
    
def main():
    # Init ROS node
    rospy.init_node('obstacle_extraction')
    
    # Create Extractor object that starts ROS subscriber and publisher
    extractor = Extractor('/scan', '/raw_obstacles', '/rack_legs')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')

if __name__ == '__main__':
    main()