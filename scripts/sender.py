#!/usr/bin/env python

import time

import rospy
from rr_cloud_bridge_analyzer.msg import Packet


def sender():
    pub = rospy.Publisher('/test/load', Packet, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    seq = 0  # incremental sequence number
    rate = rospy.Rate(20)  # hz TODO add to launch param
    while not rospy.is_shutdown():
        seq += 1
        p = Packet()
        p.seq = seq
        p.t = time.time()
        # send the packet
        pub.publish(p)
        rospy.loginfo('sent time: %lf, \t seq: %d', p.t, p.seq)
        rate.sleep()


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
