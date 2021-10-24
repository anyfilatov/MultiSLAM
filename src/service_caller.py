#!/usr/bin/env python
import sys
import rospy
import tf
from std_srvs import srv

if __name__ == '__main__':
    rospy.init_node('service_caller', anonymous=True, log_level=rospy.INFO)

    listener = tf.TransformListener()
    rate = rospy.Rate(5)
    rospy.wait_for_service('start_serialize1')
    service = rospy.ServiceProxy('/start_serialize1', srv.Empty)
    while not rospy.is_shutdown():
        listener.waitForTransform("map1", "base_link1", rospy.Time(), rospy.Duration(10000))
        listener.waitForTransform("map2", "base_link2", rospy.Time(), rospy.Duration(10000))
        time = listener.getLatestCommonTime("map1", "base_link1")
        (t1, q1) = listener.lookupTransform("map1", "base_link1", time)
        time = listener.getLatestCommonTime("map2", "base_link2")
        (t2, q2) = listener.lookupTransform("map2", "base_link2", time)

        if (t1[0] - t2[0]) ** 2 + (t1[1] - t2[1]) ** 2 < 1:
            service()
            sys.exit()

        rate.sleep()