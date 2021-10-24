#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('pose_logger', anonymous=True, log_level=rospy.INFO)
    br = tf.TransformBroadcaster()
    r = rospy.Rate(5)
    while 1:
        br.sendTransform((20, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "map1", "map2")
        r.sleep()
