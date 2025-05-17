#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, rospy, rospkg, roslaunch
from std_msgs.msg import String
from roslaunch.parent import ROSLaunchParent

class MapHandler:
    def __init__(self):
        rospy.init_node('map_handler')
        self.rp = rospkg.RosPack()
        self.runner = None
        rospy.Subscriber('/set_map', String, self.cb)
        rospy.loginfo("map_handler ready → listening on /set_map")
        rospy.spin()

    def cb(self, msg):
        name = msg.data.strip()
        pkg  = self.rp.get_path('f1tenth_stack')
        yaml = os.path.join(pkg, 'maps', name + '.yaml')
        if not os.path.isfile(yaml):
            rospy.logerr("Map not found: %s", yaml)
            return

        # ถ้ามีรัน navigation อยู่ก่อน ให้ shutdown ก่อน
        if self.runner:
            self.runner.shutdown()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = os.path.join(pkg, 'launch', 'navigation.launch')
        args = ['map_name:=' + name]
        self.runner = ROSLaunchParent(uuid, [(launch_file, args)])
        self.runner.start()
        rospy.loginfo("Launched navigation.launch with map=%s", name)

if __name__=="__main__":
    MapHandler()

