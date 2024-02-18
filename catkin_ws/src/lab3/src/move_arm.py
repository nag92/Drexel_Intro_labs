#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import JointPosition
from geometry_msgs.msg import Twist
from math import pi
from geometry_msgs.msg import Pose
from copy import deepcopy

class Turtlebot3ManipulatorController:
    
    
    def __init__(self):
        # Initialize the ROS node with an anonymous name
        rospy.init_node('turtlebot3_manipulator_controller', anonymous=True)

        # Create MoveIt! Move move_group for the Open Manipulator arm
        self.move_group = MoveGroupCommander("arm")

        # Subscribe to the joint states of the Open Manipulator
        rospy.Subscriber('/set_joint_goal', JointState, self.joint_control_callback)

        # Create a publisher to control the TurtleBot 3 base
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a publisher to control the gripper of the Open Manipulator
        self.gripper_publisher = rospy.Publisher('/open_manipulator/gripper/command', Float64, queue_size=10)

    def joint_control_callback(self, joint_goal):
        
                # Set the desired joint positions for planning
        joint_state = self.move_group.get_current_joint_values()

        joint_state = joint_goal.position
        # Plan and execute the joint motion using MoveIt!
        self.move_group.set_joint_value_target(joint_goal)
        self.move_group.go()
       
        # gripper_position_msg = Float64()
        # gripper_position_msg.data = 0.5  # Adjust the gripper position as needed
        # self.gripper_publisher.publish(gripper_position_msg)


    def go_to_pose(pose):
        """send gripper to a position

        Args:
            pose (_type_): _description_
        """
      
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()



    def plan_path():
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction




    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of the Turtlebot3ManipulatorController class
        controller = Turtlebot3ManipulatorController()

        # Run the controller
        controller.run()

    except rospy.ROSInterruptException:
        pass