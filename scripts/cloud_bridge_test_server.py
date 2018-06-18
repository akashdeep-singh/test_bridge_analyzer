#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# cloud_bridge_test_server.py
#
# Created on: Oct 25, 2017
#     Author: Yu Okamoto
# Brief: Server for cloud bridge testing

import copy

import rospy
import actionlib
from time import sleep
from std_msgs.msg import Time, Duration, Int16

from rr_cloud_bridge_analyzer.srv import CloudBridgeTestService
from rr_cloud_bridge_analyzer.msg import CloudBridgeTestActionAction, \
    CloudBridgeTestActionFeedback, CloudBridgeTestActionResult, CloudBridgeTest


class CloudBridgeTestServer(object):
    ''' subscribe topic, service and action for
        Cloud bridge test
    '''

    def __init__(self, server_id, client_id):

        self._server_id = server_id
        self._client_id = client_id
        

        #client alias name
        #todo get from param
        waiting_peer = True
        while(waiting_peer):
            topics = rospy.get_published_topics()
            for topic in topics:
                # print topic[0], 'rr_cloud_bridge_heartbeat' in topic[0]
                if 'rr_cloud_bridge_heartbeat' in topic[0] and '__self__' not in topic[0]:
                    peer = topic[0].split('/')[-1]
                    waiting_peer = False
                else:
                    print 'Waiting peer appearance'
            sleep(1)

        # ids for check message drop
        self._id_1hz = None
        self._id_10hz = None
        self._id_100hz = None

        self._drop_1hz = 0
        self._drop_10hz = 0
        self._drop_100hz = 0

        # self._topic1hz_sub = rospy.Subscriber(
        #     'topic1hz_from' + str(self._client_id), CloudBridgeTest, self._topic1hz_sub_cb
        # )
        # self._topic10hz_sub = rospy.Subscriber(
        #     'topic10hz_from' + str(self._client_id), CloudBridgeTest, self._topic10hz_sub_cb
        # )
        # self._topic100hz_sub = rospy.Subscriber(
        #     'topic100hz_from' + str(self._client_id), CloudBridgeTest, self._topic100hz_sub_cb
        # )
        
        self._srv1 = rospy.Service(
            peer+'/service_in' + str(self._server_id), CloudBridgeTestService, self._srv_cb1
        )
        self._srv2 = rospy.Service(
            'service_in' + str(self._server_id+1), CloudBridgeTestService, self._srv_cb2
        )

        self._as1 = actionlib.SimpleActionServer(
            peer+'/action_in' + str(self._server_id), CloudBridgeTestActionAction, execute_cb=self._act_cb1, auto_start=False
        )
        self._as1.start()
        self._as2 = actionlib.SimpleActionServer(
            'action_in' + str(self._server_id+1), CloudBridgeTestActionAction, execute_cb=self._act_cb2, auto_start=False
        )
        self._as2.start()
        # publishers for data measure the trasmission time
        self._trans_time_topic1hz_pub = rospy.Publisher(
            'trans_time_topic_1hz_from' + str(self._client_id), Duration, queue_size=5
        )
        self._trans_time_topic10hz_pub = rospy.Publisher(
            'trans_time_topic_10hz_from' + str(self._client_id), Duration, queue_size=5
        )
        self._trans_time_topic100hz_pub = rospy.Publisher(
            'trans_time_topic_100hz_from' + str(self._client_id), Duration, queue_size=5
        )
        self._trans_time_service_pub = rospy.Publisher(
            'trans_time_service_request_from' + str(self._client_id), Duration, queue_size=5
        )
        self._trans_time_action_pub = rospy.Publisher(
            'trans_time_action_request_from' + str(self._client_id), Duration, queue_size=5
        )

        # publishers for message drop
        self._message_drop_topic1hz_pub = rospy.Publisher(
            'drop_topic_1hz_from' + str(self._client_id), Int16, queue_size=5
        )
        self._message_drop_topic10hz_pub = rospy.Publisher(
            'drop_topic_10hz_from' + str(self._client_id), Int16, queue_size=5
        )
        self._message_drop_topic100hz_pub = rospy.Publisher(
            'drop_topic_100hz_from' + str(self._client_id), Int16, queue_size=5
        )

    def drop_check(self, new_id, prev_id, verbose=''):
        if not prev_id:
            pass
        elif new_id < prev_id:
            rospy.loginfo('id was resetted')
        elif new_id > prev_id + 1:
            rospy.logwarn('{} {} messages from id{} are dropped'.format(
                verbose, new_id - (prev_id + 1), prev_id + 1))
            return new_id - (prev_id + 1)
        return 0

    # def _topic1hz_sub_cb(self, msg):
    #     self._trans_time_topic1hz_pub.publish(rospy.Time.now() - msg.data)
    #     drop = self.drop_check(msg.id, self._id_1hz, '1hz from' + str(self._client_id))
    #     if drop:
    #         self._drop_1hz += drop
    #         self._message_drop_topic1hz_pub.publish(self._drop_1hz)
    #     self._id_1hz = msg.id

    # def _topic10hz_sub_cb(self, msg):
    #     self._trans_time_topic10hz_pub.publish(rospy.Time.now() - msg.data)
    #     drop = self.drop_check(msg.id, self._id_10hz, '10hz from' + str(self._client_id))
    #     if drop:
    #         self._drop_10hz += drop
    #         self._message_drop_topic10hz_pub.publish(self._drop_10hz)

    #     self._id_10hz = msg.id

    # def _topic100hz_sub_cb(self, msg):
    #     self._trans_time_topic100hz_pub.publish(rospy.Time.now() - msg.data)
    #     drop = self.drop_check(msg.id, self._id_100hz, '100hz from' + str(self._client_id))
    #     if drop:
    #         self._drop_100hz += drop
    #         self._message_drop_topic100hz_pub.publish(self._drop_100hz)
    #     self._id_100hz = msg.id

    def _srv_cb1(self, msg):
        self._trans_time_service_pub.publish(rospy.Time.now() - msg.data)
        return rospy.Time.now()

    def _srv_cb2(self, msg):
        self._trans_time_service_pub.publish(rospy.Time.now() - msg.data)
        return rospy.Time.now()

    def _act_cb1(self, goal):
        self._trans_time_action_pub.publish(rospy.Time.now() - goal.data)
        self._as1.set_succeeded(CloudBridgeTestActionResult(data=rospy.Time.now()))

    def _act_cb2(self, goal):
        self._trans_time_action_pub.publish(rospy.Time.now() - goal.data)
        self._as2.set_succeeded(CloudBridgeTestActionResult(data=rospy.Time.now()))

def main():
    rospy.init_node('cloud_bridge_test_server')
    server_id = rospy.get_param('~server_id', 1)
    client_id = rospy.get_param('~client_id', 2)
    node = CloudBridgeTestServer(server_id, client_id)
    rospy.loginfo('Start Cloud Bridge Test Server...')
    rospy.spin()

if __name__ == '__main__':
    main()
