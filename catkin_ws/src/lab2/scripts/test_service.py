#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetPlanResponse

start  = PoseStamped()
def a_star_client(x, y):
    rospy.wait_for_service("a_star")
    rospy.loginfo("senings")
    try:
        s = rospy.ServiceProxy('a_star', GetPlan)
        resp1 = s(x, y,0)
        return resp1.plan
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def start_callback(msg):
    start.pose = msg.pose.pose
    start.header = msg.header

def goal_callback(goal):
    a_star_client(start, goal)


if __name__ == "__main__":
    rospy.init_node("test_service")
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, start_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.spin()