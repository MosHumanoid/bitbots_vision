#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from bitbots_vision.cfg import VisionConfig

def callback(config, level):
    rospy.loginfo("sth changed")
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    rospy.logerr("start srv")
    srv = Server(VisionConfig, callback)
    rospy.logerr("end srv")
    rospy.spin()
