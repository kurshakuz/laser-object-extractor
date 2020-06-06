#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped

def path_reducer(data, args):
    # print(data)

    step = round(float(args["total_path_num"])/args["desired_path_num"])
    if ((args["total_path_counter"]%step) == 0):
        args["publisher"].publish(data) 
        print(data)
        args["desired_path_counter"] += 1

    args["total_path_counter"] += 1
    
def main():
    # Init ROS node
    rospy.init_node('path_reduction')
       
    # Default values in case params are not set
    desired_path_num = 100
    total_path_num = 992

    # Read ROS params
    if rospy.has_param('~desired_path_num'):
        desired_path_num = rospy.get_param("~desired_path_num")
    # print(type(desired_path_num))

    publisher = rospy.Publisher('/vslam2d_pose_reduced', PoseStamped, queue_size=10)

    args_dict = {
        "total_path_counter": 0,
        "desired_path_counter": 0,
        "desired_path_num": desired_path_num, 
        "total_path_num": total_path_num, 
        "publisher": publisher
    }

    rospy.Subscriber("/vslam2d_pose", PoseStamped, path_reducer, args_dict)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')

    print("desired_path_counter: " + str(args_dict["desired_path_counter"]))
    print("total_path_counter: " + str(args_dict["total_path_counter"]))

if __name__ == '__main__':
    main()