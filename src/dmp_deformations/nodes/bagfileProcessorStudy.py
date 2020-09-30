#!/usr/bin/env python

""" Process the study data for amount of input
and idle time!!
 Created: 09/29/2020
"""

__author__ = "Mike Hagenow"

import numpy as np

from LFD_helpers.bagfile_reader import bagfile_reader
from LFD_helpers.ros_data_type_helpers import pose_array_to_np_array, fd_to_np_array
import mayavi.mlab as mlab
from matplotlib import pyplot as plt
from LFD_helpers.algorithms.filters import butter_lowpass_filter

# This is the main routine that loads recorded demonstration files
# and the ultimately calls to construct the DMPs for robot replay
def processData(file):
    bfr = bagfile_reader(file)

    fd_input, fd_timesamples =  bfr.get_topic_msgs("/fd/input")
    hand_input, hand_timesamples = bfr.get_topic_msgs("/vrpn_client_node/HandTracker/pose")
    p_hand_input, q_hand_input = pose_array_to_np_array(hand_input)
    p_fd_input = fd_to_np_array(fd_input)


    print(np.shape(p_hand_input))


    if (np.shape(fd_input)[0] > 0):
        fd_sample_time = np.mean(np.diff(fd_timesamples)) * 1.0  # both have same sample time

        fig = mlab.figure()
        mlab.plot3d(p_fd_input[:, 0], p_fd_input[:, 1], p_fd_input[:, 2], color=(0, 1, 0), tube_radius=0.001,
                    tube_sides=20)
        mlab.show()
        v_fd = (1 / fd_sample_time) * np.linalg.norm(np.diff(p_fd_input, axis=0), axis=1)
        v_fd_filt = butter_lowpass_filter(v_fd, 0.8, 1.0 / fd_sample_time, order=4, axis=0)
        plt.plot(v_fd,'b')
        plt.plot(v_fd_filt,'r')

    elif (np.shape(p_hand_input)[0] > 0):
        hand_sample_time = np.mean(np.diff(hand_timesamples)) * 1.0  # both have same sample time

        fig = mlab.figure()
        mlab.plot3d(p_hand_input[:, 0], p_hand_input[:, 1], p_hand_input[:, 2], color=(1, 0, 0), tube_radius=0.001,
                    tube_sides=20)
        mlab.show()
        v_hand = (1 / hand_sample_time) * np.linalg.norm(np.diff(p_hand_input, axis=0), axis=1)
        v_hand_filt = butter_lowpass_filter(v_hand, 0.9, 1.0 / hand_sample_time, order=4, axis=0)
        plt.plot(v_hand,'b')
        plt.plot(v_hand_filt,'r')





    print("YO")





    # wrench_sample_time = np.mean(np.diff(wrench_t_upper)) * 1.0  # both have same sample time
    #
    # wrenches_sensor_values_upper = butter_lowpass_filter(wrenches_sensor_values_upper,
    #                                                      mocap_sample_time / wrench_sample_time,
    #                                                      1.0 / wrench_sample_time, order=4, axis=0)



def main():
    # processData('/home/mike/Desktop/trial_210849.bag')
    processData('/home/mike/Documents/rf/trial_193917.bag')

if __name__ == "__main__":
    main()




