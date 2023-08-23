#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SendGoalMB:
    def __init__(self):
        # clients
        rospy.loginfo("waiting for move_base server")
        self.mb_clinet = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.mb_clinet.wait_for_server()
        self.mb_goal_state = False

    def send_goal(self, x,y,yaw):
        self.mb_goal_state = False
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.mb_clinet.send_goal(goal, done_cb=self.mb_cone_cb)

    def mb_cone_cb(self, state, result):
        print(state)
        print(result)


if __name__ == "__main__":

    rospy.init_node("send_goal_mb")

    x = 1.5
    y = 1.3
    yaw = 0

    sg = SendGoalMB()
    sg.send_goal(x, y, yaw)
    rospy.spin()

