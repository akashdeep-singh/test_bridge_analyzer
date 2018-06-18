#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# cloud_bridge_test_client.py
#
# Created on: Oct 25, 2017
#     Author: Yu Okamoto
# Brief: Client for cloud bridge testing

import rospy
import actionlib
from std_msgs.msg import Time, Duration
from time import sleep
from rr_cloud_bridge_analyzer.srv import CloudBridgeTestService
from rr_cloud_bridge_analyzer.msg import CloudBridgeTestActionAction, \
    CloudBridgeTestActionFeedback, CloudBridgeTestActionResult, CloudBridgeTest


class CloudBridgeTestClient(object):
    ''' publish topic, service and action for Cloud bridge test
    '''

    def __init__(self, server_id, client_id):
        self._server_id = server_id
        self._client_id = client_id

        # self._topic1hz_pub = rospy.Publisher(
        #     'topic1hz_from' + str(self._client_id), CloudBridgeTest, queue_size=5
        # )
        # self._topic10hz_pub = rospy.Publisher(
        #     'topic10hz_from' + str(self._client_id), CloudBridgeTest, queue_size=5
        # )
        # self._topic100hz_pub = rospy.Publisher(
        #     'topic100hz_from' + str(self._client_id), CloudBridgeTest, queue_size=5
        # )

        # publishers for data measure the trasmission time
        self._trans_time_service_pub = rospy.Publisher(
            'trans_time_service_result_from' + str(self._server_id), Duration, queue_size=5
        )
        self._trans_time_action_pub = rospy.Publisher(
            'trans_time_action_result_from' + str(self._server_id), Duration, queue_size=5
        )

    def spin(self, freq):
        
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


        r = rospy.Rate(freq)
        count = 0
        id_1hz = 0
        id_10hz = 0
        id_100hz = 0
        rospy.loginfo('Start Cloud Bridge Test Client...')
        while not rospy.is_shutdown():
            # topic = CloudBridgeTest()
            # topic.id = id_100hz
            # topic.data = rospy.Time.now()  # '100hz from ' + str(self._client_id)
            # id_100hz += 1
            # self._topic100hz_pub.publish(topic)
            # if count % 10 == 0:
            #     # topic.header.stamp = 3
            #     topic.id = id_10hz
            #     topic.data = rospy.Time.now()  # '10hz from ' + str(self._client_id)
            #     id_10hz += 1
            #     self._topic10hz_pub.publish(topic)
            if count % 100 == 0:
                # topic.id = id_1hz
                # topic.data = rospy.Time.now()  # '1hz from ' + str(self._client_id)
                # id_1hz += 1
                # self._topic1hz_pub.publish(topic)

                # service
                # print  'service_in' + str(self._server_id)
                rospy.wait_for_service('service_in' + str(self._server_id))
                srv_client = rospy.ServiceProxy(
                    'service_in' + str(self._server_id), CloudBridgeTestService)
                t = srv_client(rospy.Time.now()).data
                self._trans_time_service_pub.publish(rospy.Time.now() - t)
                rospy.loginfo('Get service result:'+str(srv_client(rospy.Time.now())))

                # print  peer+'service_in' + str(self._server_id+1)
                rospy.wait_for_service(peer+'/service_in' + str(self._server_id+1))
                srv_client = rospy.ServiceProxy(
                    peer+'/service_in' + str(self._server_id+1), CloudBridgeTestService)
                t = srv_client(rospy.Time.now()).data
                self._trans_time_service_pub.publish(rospy.Time.now() - t)
                rospy.loginfo('Get service result:'+str(srv_client(rospy.Time.now())))


                # # action
                # print 'action_in' + str(self._server_id) 
                act_client = actionlib.SimpleActionClient('action_in' + str(self._server_id),
                                                          CloudBridgeTestActionAction)
                act_client.wait_for_server()
                goal = Time(data=rospy.Time.now())
                act_client.send_goal(goal)
                act_client.wait_for_result()
                t = act_client.get_result().data
                self._trans_time_action_pub.publish(rospy.Time.now() - act_client.get_result().data)
                rospy.loginfo('Get action result:'+str(act_client.get_result()))

                # print peer+'action_in' + str(self._server_id+1)
                act_client = actionlib.SimpleActionClient(peer+'/action_in' + str(self._server_id+1),
                                                          CloudBridgeTestActionAction)
                act_client.wait_for_server()
                goal = Time(data=rospy.Time.now())
                act_client.send_goal(goal)
                act_client.wait_for_result()
                t = act_client.get_result().data
                self._trans_time_action_pub.publish(rospy.Time.now() - act_client.get_result().data)
                rospy.loginfo('Get action result:'+str(act_client.get_result()))



            if count > 1000:  # todo temporary reset num
                count = 0

            count += 1
            r.sleep()


def main():
    rospy.init_node('cloud_bridge_test_client')
    server_id = rospy.get_param('~server_id', 1)
    client_id = rospy.get_param('~client_id', 2)
    node = CloudBridgeTestClient(server_id, client_id)
    node.spin(100)


if __name__ == '__main__':
    main()
