#! /usr/bin/env python

""" Routine computes transformation between two rigid frames by use of optimization between
two sets of data in the frames.

 Last Updated: 02/18/2019
"""


from rigid_registration import rigid_registration_from_svd
from bagfile_reader import bagfile_reader
import numpy as np
import yaml
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

def main():
    bagfile='/home/mike/Desktop/calib_8-8_1.bag'
    calculate(bagfile)



def calculate(bagfile):
    br = bagfile_reader(bagfile)
    # js, jt = br.get_topic_msgs("/robot/joint_states")
    tf_msgs, tf_timestamps  = br.get_topic_msgs("/tf")

    if(len(tf_msgs)==0):
        print "No TF messages to compute registration"
        return

    optical_breadboard = []
    optical_breadboard_q = []
    panda_frame = []
    panda_frame_ts = []
    panda_mocap = []
    panda_mocap_ts = []


    for msg,ts in zip(tf_msgs,tf_timestamps):
        for transform in msg.transforms:
            if transform.child_frame_id=='end_effector':
                panda_frame.append([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
                panda_frame_ts.append(ts)
            elif transform.child_frame_id=='optical_breadboard':
                optical_breadboard.append([transform.transform.translation.x, transform.transform.translation.y,
                                    transform.transform.translation.z])
                optical_breadboard_q.append([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            elif transform.child_frame_id=='panda_mocap':
                panda_mocap.append([transform.transform.translation.x, transform.transform.translation.y,
                                    transform.transform.translation.z])
                panda_mocap_ts.append(ts)


    # Interpolate everything with respect to the timestamps for the panda motion capture
    panda_mocap = np.array(panda_mocap)
    panda_frame_interp = interp1d(panda_frame_ts, panda_frame,
                                     axis=0, fill_value='extrapolate', bounds_error=False)(panda_mocap_ts)


    # #Uses SVD algorithm to optimize transform from two sets of data
    r_est_svd, t_est_svd = rigid_registration_from_svd(panda_mocap, panda_frame_interp)
    #
    # #Quaternions from 4x4 rotation matrix
    A_mocap_panda = np.zeros((4, 4))
    A_mocap_panda[:3, :3] = r_est_svd
    A_mocap_panda[3, 3] = 1
    A_mocap_panda[:3,3] = t_est_svd

    # Find breadboard in the mocap scene
    t_breadboard_mocap = np.average(optical_breadboard,axis=0)
    R_breadboard_mocap = R.from_quat(optical_breadboard_q[0]).as_dcm() # use first, ideal would be average

    A_breadboard_mocap = np.zeros((4, 4))
    A_breadboard_mocap[:3, :3] = R_breadboard_mocap
    A_breadboard_mocap[3, 3] = 1
    A_breadboard_mocap[:3, 3] = t_breadboard_mocap

    # Calculate the transition between the breadboard and the panda_frame
    A_breadboard_panda = np.matmul(A_mocap_panda,A_breadboard_mocap)


    print("A mocap panda", A_mocap_panda)
    print("--")
    print("--")
    print("--")
    print("A breadboard_panda", A_breadboard_mocap)
    print("--")
    print("--")
    print("--")
    print("A_breadboard_panda",A_breadboard_panda)


if __name__ == '__main__':
    main()
