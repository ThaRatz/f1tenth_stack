#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, os
from std_msgs.msg import String

def main():
    rospy.init_node('map_list_publisher')
    map_dir = rospy.get_param('~map_dir', '')
    pub     = rospy.Publisher('/map_list', String, queue_size=1, latch=True)
    if not os.path.isdir(map_dir):
        rospy.logerr("map_dir not found: %s", map_dir)
        return
    names = [os.path.splitext(f)[0] for f in os.listdir(map_dir) if f.endswith('.yaml')]
    if not names:
        rospy.logwarn("No .yaml maps in %s", map_dir)
    for n in names:
        pub.publish(n)
        rospy.loginfo("Published map: %s", n)
    rospy.spin()

if __name__=='__main__':
    main()

