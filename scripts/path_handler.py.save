#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, actionlib
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PathHandler:
    def __init__(self):
        rospy.init_node('path_handler')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")
        rospy.Subscriber('/planned_path', Path, self.cb)
        rospy.loginfo("path_handler ready → listening on /planned_path")
        rospy.spin()

    def cb(self, msg):
        rospy.loginfo("Received path with %d poses", len(msg.poses))
        for pose in msg.poses:
            goal = MoveBaseGoal()
            goal.target_pose = pose
            goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo("→ Sending goal: (%.2f, %.2f)",
                          pose.pose.position.x, pose.pose.position.y)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.loginfo("  result: %s", self.client.get_result())

if __name__=="__main__":
    PathHandler()

