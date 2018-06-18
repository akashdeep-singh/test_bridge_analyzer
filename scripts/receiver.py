#!/usr/bin/env python

import signal
import sys
import time

import matplotlib.pyplot as plt
import rospy
from rr_cloud_bridge_analyzer.msg import Packet

plt.ion()


class Receiver(object):
    def __init__(self):
        # dictionary
        self.seq_list = []
        self.st_list = []
        self.rt_list = []
        self.dt_list = []
        signal.signal(signal.SIGINT, self.signal_handler)

    def create_plot(self):
        # setup plot TODO enlarge figure size 
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([], [], 'bo-')

        # autoscale TODO enable autoscale for min/max
        self.ax.set_autoscaley_on(True)
        self.ax.grid()

        plt.xlabel('Sequence number')
        plt.ylabel('Latency(s)')
        plt.title('Latency vs. Sequence number')

    def update_plot(self, xdata, ydata):
        # update data 
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        self.ax.relim()
        self.ax.autoscale_view()

        # draw 
        self.figure.canvas.draw()
        # no flush?
        # self.figure.canvas.flush_events()

    def callback(self, data):
        t = time.time()
        dt = t - data.t
        if dt < 0:
            rospy.logerr('sender and receiver are not time-synced!')
        rospy.loginfo('recv time(s): %lf, \t seq: %d, \t diff(s): %lf',
                      time.time(), data.seq, dt)

        self.seq_list.append(data.seq)
        self.st_list.append(data.t)
        self.rt_list.append(t)
        self.dt_list.append(dt)

    def signal_handler(self, signal, frame):
        print('Pressed Ctrl+C!')
        sys.exit(0)

    def __call__(self):
        self.create_plot()

        rospy.init_node('receiver', anonymous=True)
        # subscribe to the topic
        rospy.Subscriber("/test/load", Packet, self.callback)

        # rospy.spin()
        start = time.time()
        while True:
            # replot every second
            if time.time() - start > 1:
                # TODO plot others next to it in different color
                self.update_plot(self.seq_list, self.dt_list)
                start = time.time()
            else:
                pass


if __name__ == '__main__':
    r = Receiver()
    r()
