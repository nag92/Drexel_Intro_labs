#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, PoseStamped
import copy
import math
import numpy as np
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
class myTurtle():
    
    
    def __init__(self) -> None:
        self._odom_list = tf.TransformListener()
        rospy.Subscriber("odom", Odometry, self.odom_cb)
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/goal', PoseStamped, self.nav_to_pose, queue_size=1)
        self._pose = Pose()
        self._velocity = Twist()
        

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        rospy.loginfo(goal)
        now = rospy.Time.now()
        rospy.loginfo("Called")
        
        rate = rospy.Rate(10.0)
        
        
        

            
            
            
                
        self._odom_list.waitForTransform('base_footprint', 'odom', now, rospy.Duration(1))
        trans_goal = self._odom_list.transformPose('base_footprint', goal)

        (goal_angle_roll, goal_angle_pitch, goal_angle_yaw) = tf.transformations.euler_from_quaternion(
                                                                                [trans_goal.pose.orientation.x,
                                                                                 trans_goal.pose.orientation.y,
                                                                                 trans_goal.pose.orientation.z,
                                                                                 trans_goal.pose.orientation.w])

            
   
        
        
        yaw = self.convert_to_euler(trans_goal.pose.orientation)

        goal_distance = math.sqrt(trans_goal.pose.position.x ** 2 + trans_goal.pose.position.y ** 2)
        heading_angle =  math.atan2(trans_goal.pose.position.y, trans_goal.pose.position.x)

        # move through the points
        rospy.loginfo("aligning")
        self.rotate(heading_angle)
        rospy.loginfo("going straight")
        self.drive_straight(dist=goal_distance,vel=0.5)
        rospy.loginfo("turngin")
        self.rotate(goal_angle_yaw - heading_angle)
        self.stop()


    def odom_cb(self,msg:Odometry) ->None:
        """_summary_

        Args:
            msg (Odometry): _description_
        """
        self._pose = msg.pose
        self._velocity.linear = msg.twist.twist.linear
        self._velocity.angular = msg.twist.twist.angular
    
    def stop(self)->None:
        """_summary_
        """
        drive_msg = Twist()
        self._vel_pub.publish(drive_msg)
        
    def drive_straight(self, dist: float, vel: float)->None:
        """_summary_

        Args:
            dist (_type_): _description_
        """
        
        drive_msg = Twist()

        time = dist / vel
        rospy.loginfo(time)
        origin = copy.deepcopy(self._pose)
        direction_correction = np.sign(vel)
        while self.displacement(origin.pose, self._pose.pose) < dist:
            drive_msg.linear.x = vel #* direction_correction
            self._vel_pub.publish(drive_msg)

        self.stop()
        
    
    def spin_wheels(self, u1, u2, time):
        """
        Spin the two wheels

        :param u1: wheel 1 speed
        :param u2: wheel 2 speed
        :param time: time to drive
        :return:
        """

        drive_msg = Twist()

        # use differential drive kinematics to compute V and omega (linear x = V, angular z = omega)
        diameter = 0.160
        vel = 0.5 * (u1 + u2)
        omega = (u2 - u1) / diameter
        drive_msg.linear.x = vel
        drive_msg.angular.z = omega
        drive_start_time = rospy.Time.now().secs
        while rospy.Time.now().secs - drive_start_time < time:
            self._vel_pub.publish(drive_msg)

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """

        max_omega = 0.5
        vel = Twist()
        rospy.loginfo(type(self._pose))
        origin = PoseStamped()
        origin = copy.deepcopy(self._pose)
        rospy.logerr(isinstance(origin,PoseStamped))

        yaw = self.convert_to_euler(origin.pose.orientation)
        init_ang = yaw

        direction_correction = np.sign(angle)
        angle = abs(angle)

        while abs(yaw - init_ang) < angle:
            yaw = self.convert_to_euler(self._pose.pose.orientation)
            vel.angular.z = direction_correction * max_omega
            self._vel_pub.publish(vel)


        self._vel_pub.publish(Twist())
    
    def displacement(self, ref_point, current_point):
        # type: (Pose, Pose) -> float
        return math.sqrt((current_point.position.x - ref_point.position.x) ** 2 + (current_point.position.y - ref_point.position.y) ** 2)
    
    def convert_to_euler(self, quat):
        # type: (Quaternion) -> float
        """

        :param quat:
        :return:
        """

        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw


def main():
    rospy.init_node("my_turtle")
    turtle = myTurtle()
    rospy.spin()
    
    




if __name__ == '__main__':
    main()