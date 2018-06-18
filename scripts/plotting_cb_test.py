#!/usr/bin/env python

import os
import numpy as np
import matplotlib.pyplot as plt

from rr_tools_plotting.plottable_objects import DurationObject
from rr_tools_plotting.plottable_objects import IntObject

from rr_tools_plotting.plotting_tools import PlottingTools

from rr_tools_plotting.plotting_handler import extract_bag_data, get_bag_handle, catch_flag


def main():
    """Example usage of plotting tools."""

    # catch flag
    flag_val = catch_flag()

    # bag path
    full_path_to_bag = flag_val.bag_file

    # data holder
    trans_time_action_request_from1 = DurationObject('/trans_time_action_request_from1')
    trans_time_action_request_from2 = DurationObject('/trans_time_action_request_from2')
    trans_time_action_result_from1 = DurationObject('/trans_time_action_result_from1')
    trans_time_action_result_from2 = DurationObject('/trans_time_action_result_from2')
    trans_time_service_request_from1 = DurationObject('/trans_time_service_request_from1')
    trans_time_service_request_from2 = DurationObject('/trans_time_service_request_from2')
    trans_time_service_result_from1 = DurationObject('/trans_time_service_result_from1')
    trans_time_service_result_from2 = DurationObject('/trans_time_service_result_from2')
    trans_time_topic_100hz_from1 = DurationObject('/trans_time_topic_100hz_from1')
    trans_time_topic_100hz_from2 = DurationObject('/trans_time_topic_100hz_from2')
    trans_time_topic_10hz_from1 = DurationObject('/trans_time_topic_10hz_from1')
    trans_time_topic_10hz_from2 = DurationObject('/trans_time_topic_10hz_from2')
    trans_time_topic_1hz_from1 = DurationObject('/trans_time_topic_1hz_from1')
    trans_time_topic_1hz_from2 = DurationObject('/trans_time_topic_1hz_from2')
    drop_topic_100hz_from1 = IntObject('/drop_topic_100hz_from1')
    drop_topic_100hz_from2 = IntObject('/drop_topic_100hz_from2')
    drop_topic_10hz_from1 = IntObject('/drop_topic_10hz_from1')
    drop_topic_10hz_from2 = IntObject('/drop_topic_10hz_from2')
    drop_topic_1hz_from1 = IntObject('/drop_topic_1hz_from1')
    drop_topic_1hz_from2 = IntObject('/drop_topic_1hz_from2')

    # extract data
    bag = get_bag_handle(full_path_to_bag)    # bag handle
    holder_list = [
        trans_time_action_request_from1,
        trans_time_action_request_from2,
        trans_time_action_result_from1,
        trans_time_action_result_from2,
        trans_time_service_request_from1,
        trans_time_service_request_from2,
        trans_time_service_result_from1,
        trans_time_service_result_from2,
        trans_time_topic_100hz_from1,
        trans_time_topic_100hz_from2,
        trans_time_topic_10hz_from1,
        trans_time_topic_10hz_from2,
        trans_time_topic_1hz_from1,
        trans_time_topic_1hz_from2,
        drop_topic_100hz_from1,
        drop_topic_100hz_from2,
        drop_topic_10hz_from1,
        drop_topic_10hz_from2,
        drop_topic_1hz_from1,
        drop_topic_1hz_from2,
    ]
    extract_bag_data(holder_list, bag,
                     start_time=flag_val.start_time, end_time=flag_val.end_time, use_topic_timestamp=False)
    bag.close()

    # plotting tools
    pt = PlottingTools()
    pt.x_limit = flag_val.x_range    # x-axis range (default=[], uses automatic range)
    pt.y_limit_large = flag_val.y_range   # y-axis range for value field
    pt.y_limit_small = flag_val.y_range_state

    # fill plotting data holder (for usage of each property, see plotting_holder.py)
    # pt.position = [(navest.t,navest._lat,navest._long,navest._alt, 'lla est', 'r-')]

    # save figure
    save_fig = False
    if flag_val.output_path is not None:
        save_fig = True
        fig_path = flag_val.output_path
        if not os.path.exists(fig_path):  # create output directory if necessary
            os.makedirs(fig_path)

    # calculate average
    for data in holder_list[0:14]:
        sum = 0
        if len(data.t):
            for i in range(len(data.t)):
                sum += data._data_ns[i]
            print '{:<33} average delay {:>.0} [nsec]'.format(data._topic, sum / len(data.t))

    for data in holder_list[14:21]:
        if data._data:
            print '{:<33} message drop {:>} [message]'.format(data._topic, data._data[-1])

    for (trans, drop) in zip(holder_list[8:14], holder_list[14:21]):
        if drop._data:
            print '{:<33} message drop {:>} [message]'.format(drop._topic, drop._data[-1])
            print '{:>33} message  num {:>} [message]'.format('', drop._data[-1] + len(trans.t))
            print '{:>33} drop ratio   {:>} [%]'.format('', 100.0 * drop._data[-1] / (drop._data[-1] + len(trans.t)))

    # xmin = 0
    # xmax = 430
    # plot figure############################
    fig_num = 1
    fig_idx = 1
    plt.figure("Duration", figsize=(16.0, 10.0))
    plt.subplot(fig_num, 1, fig_idx)
    plt.plot(trans_time_topic_100hz_from2.t, trans_time_topic_100hz_from2._data_ns, label='')
    plt.ylabel("Transmission delay[nsec]")
    plt.xlabel("t[sec]")
    plt.legend()
    # #plt.xlim(xmin, xmax)
    fig_idx += 1

    if save_fig:
        f_name = os.path.join(fig_path, "position.png")
        plt.savefig(f_name)
        print "figure saved to " + f_name

    # display figure
    plt.show()


if __name__ == '__main__':
    """Main function."""
    main()
