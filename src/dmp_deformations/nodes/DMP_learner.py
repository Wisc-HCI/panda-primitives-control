#!/usr/bin/env python

""" Calculates DMP using LWR
 from a presegmented demonstration
 e.g., generate_path_helper, LFD_Hybrid
 Created: 06/08/2020
"""

__author__ = "Mike Hagenow"

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import mayavi.mlab as mlab
import os
import rospy
from DMP import DMP
from scipy import signal
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import csv
from scipy.interpolate import interp1d
from scipy.ndimage import interpolation
from dtw import dtw
import PyBSpline
from deformation_static_scaling import getStaticScaling
import rospkg

demonstrations = []

def loadPresegmentedCore(file):
    rospack = rospkg.RosPack()
    directory = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations"
    demonstration_data = []

    # Skip over lines for surfaces, segmentation samples, interaction, and variance
    demonstration_data.append(np.loadtxt(directory+'/'+file, delimiter=",", skiprows=4))
    segmentation = []

    # Zip doesn't work if there is 1 entry since it is not iterable
    if(np.shape(np.loadtxt(directory+'/'+file, delimiter=",",max_rows=1,skiprows=1))!=()):
        for surface_model,point,interaction,const_variance in zip(np.genfromtxt(directory+'/'+file,delimiter=",",max_rows=1,dtype='str') ,np.loadtxt(directory+'/'+file, delimiter=",",max_rows=1, skiprows=1),np.loadtxt(directory+'/'+file, delimiter=",",max_rows=1, skiprows=2),np.genfromtxt(directory+'/'+file,delimiter=",",max_rows=1,skip_header=3,dtype='str')):
            # right now only allows for position-level

            temp_var = np.array(const_variance.split(" "))

            if interaction==1:
                # TODO: fix this!!!!
                segmentation.append((point, temp_var, np.array([1, 1, 1, 1, 1, 1, 1]),surface_model))
            else:
                segmentation.append((point, temp_var, np.array([1, 1, 0, 1, 1, 1, 1]),surface_model))
    else:
        surface_model = np.genfromtxt(directory+'/'+file,delimiter=",",max_rows=1,dtype='str')
        point = np.loadtxt(directory+'/'+file, delimiter=",",max_rows=1, skiprows=1)
        interaction = np.loadtxt(directory+'/'+file, delimiter=",",max_rows=1, skiprows=2)
        if interaction == 1:
            # TODO: fix this!!!!
            segmentation.append((point, np.array([1, 1, 1, 1, 1, 1, 1]),surface_model))
        else:
            segmentation.append((point, np.array([1, 1, 0, 1, 1, 1, 1]),surface_model))


    # Use Dynamic Time Warping to figure out the equivalent
    # index scaling for each of the additional demonstrations

    # DTW is just done with the x,y,and z-values for now
    # only relevant anyways when doing LFD with multiple demonstrations

    x = []
    for ii in range(0,np.shape(demonstration_data[0])[0]):
        # Features are stored as lists of tuples
        x.append((demonstration_data[0][ii,0],demonstration_data[0][ii,1],demonstration_data[0][ii,2]))

    alignment_curves = []
    for ii in range(0,len(demonstration_data)):
        y = []
        # compare each of the demos to the first which is used for segmentation
        manhattan_distance = lambda x, y: np.abs(x[0] - y[0])+np.abs(x[1] - y[1])+np.abs(x[2] - y[2])

        for jj in range(0, np.shape(demonstration_data[ii])[0]):
            # Features are stored as lists of tuples
            y.append((demonstration_data[ii][jj, 0], demonstration_data[ii][jj, 1], demonstration_data[ii][jj, 2]))

        d, cost_matrix, acc_cost_matrix, path = dtw(x, y, dist=manhattan_distance)
        alignment_curves.append((path[0],path[1]))

    calculateDMP(demonstration_data, segmentation, alignment_curves)
    print "..."
    print "..."
    print "LOAD AND CALCULATIONS COMPLETE"

# Used to remove the struct from the input for troubleshooting
def loadPresegmented(data):
    loadPresegmentedCore(data.data)

def calculateDMP(demonstration_data, segmentation, alignment_curves):
    print "Building DMPs"
    dmps = []
    dmps_reverse = []

    variance_per_segment=[]

    # Write a CSV with the final trajectory which can be read and executed
    with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/learneddmp.csv', 'w') as csvfile:
        # Write to file for potential replay
        # print "Writing to DMP file"

        demonstration_data_trimmed = []
        trajectories_plotting=[]

        # For each of the DMPs
        for xx in range(0,len(segmentation)):
            segment = segmentation[xx]

            # Create a DMP
            dmp = DMP()
            dmps.append(dmp)

            # Create a DMP for the reverse behavior
            dmp_r = DMP()
            dmps_reverse.append(dmp_r)

            sel_vec = segment[2]
            variances = segment[1]
            surface=segment[3]

            demonstration_per_dmp = []
            reverse_demonstration_per_dmp = []

            # 3 Deformation Directions need calculations of variances
            var_x_temp = np.zeros((200, len(demonstration_data)))
            var_y_temp = np.zeros((200, len(demonstration_data)))
            var_z_temp = np.zeros((200, len(demonstration_data)))
            
            # Get the DMP sections for each of the demonstrations
            for yy in range(0,len(demonstration_data)):
                demo = demonstration_data[yy]

                 # Get Start and End Points
                start_index = segment[0]
                start_index = alignment_curves[yy][1][int(np.round(np.median(np.where(alignment_curves[yy][0]==start_index))))]
                end_index = len(demo)

                # print "LEN:",np.shape(demo)

                # If not the last segment, get the next event index
                if xx+1<len(segmentation):
                    end_index = segmentation[xx+1][0]
                    end_index = alignment_curves[yy][1][
                        int(np.round(np.median(np.where(alignment_curves[yy][0] == end_index))))]

                temp = np.zeros((end_index-start_index,7))
                temp_reverse = np.zeros((end_index-start_index,7))

                # Get either forces or positions depending on selection vector
                # First three are positions, 7-9 are forces (after quaternion)
                temp[:, 0] = demo[start_index:end_index,0+7*(1-sel_vec[0])].reshape((end_index-start_index,))
                temp[:, 1] = demo[start_index:end_index,1+7*(1-sel_vec[1])].reshape((end_index-start_index,))
                temp[:, 2] = demo[start_index:end_index,2+7*(1-sel_vec[2])].reshape((end_index-start_index,))
                temp[:, 3] = demo[start_index:end_index,3].reshape((end_index - start_index,))
                temp[:, 4] = demo[start_index:end_index,4].reshape((end_index - start_index,))
                temp[:, 5] = demo[start_index:end_index,5].reshape((end_index - start_index,))
                temp[:, 6] = demo[start_index:end_index,6].reshape((end_index - start_index,))

                # Get the same things in reverse order to be learned for the reversible analog of the DMP!
                temp_reverse[:, 0] = np.flip(demo[start_index:end_index, 0 + 7 * (1 - sel_vec[0])]).reshape((end_index - start_index,))
                temp_reverse[:, 1] = np.flip(demo[start_index:end_index, 1 + 7 * (1 - sel_vec[1])]).reshape((end_index - start_index,))
                temp_reverse[:, 2] = np.flip(demo[start_index:end_index, 2 + 7 * (1 - sel_vec[2])]).reshape((end_index - start_index,))
                temp_reverse[:, 3] = np.flip(demo[start_index:end_index, 3]).reshape((end_index - start_index,))
                temp_reverse[:, 4] = np.flip(demo[start_index:end_index, 4]).reshape((end_index - start_index,))
                temp_reverse[:, 5] = np.flip(demo[start_index:end_index, 5]).reshape((end_index - start_index,))
                temp_reverse[:, 6] = np.flip(demo[start_index:end_index, 6]).reshape((end_index - start_index,))


                # ADD TO VARIANCE CALCULATION
                # TODO: does this work whatsoever?
                var_x_temp[:,yy] = interpolation.zoom(demo[start_index:end_index,0 + 7 * (1 - sel_vec[0])],200.0/(end_index-start_index))
                var_y_temp[:,yy] = interpolation.zoom(demo[start_index:end_index,1 + 7 * (1 - sel_vec[1])],200.0/(end_index-start_index))
                var_z_temp[:,yy] = interpolation.zoom(demo[start_index:end_index,2 + 7 * (1 - sel_vec[2])],200.0/(end_index-start_index))

                
                demonstration_per_dmp.append(temp)
                reverse_demonstration_per_dmp.append(temp_reverse)


            # Calculate the variances and store for later
            std_dev_x = np.sqrt(np.var(var_x_temp,axis=1))
            std_dev_y = np.sqrt(np.var(var_y_temp,axis=1))
            std_dev_z = np.sqrt(np.var(var_z_temp,axis=1))

            # Apply the scaling algorithm
            # std_dev_x = getStaticScaling(np.linspace(0, 1, len(std_dev_x)), std_dev_x)
            # std_dev_y = getStaticScaling(np.linspace(0, 1, len(std_dev_y)), std_dev_y)
            # std_dev_z = getStaticScaling(np.linspace(0, 1, len(std_dev_z)), std_dev_z)

            variance_per_segment.append((std_dev_x,std_dev_y,std_dev_z))

            dmps[xx].inputData(demonstration_data=demonstration_per_dmp)
            dmps[xx].computeDMP()
            dmps_reverse[xx].inputData(demonstration_data=reverse_demonstration_per_dmp)
            dmps_reverse[xx].computeDMP()

            trajectories = dmps[xx].getTrajectory()

            # Figure out how long the demonstration should be based on max difference (velocity)
            # in kinematic directions and interpolate such that this is sent at 1000 Hz.

            #########################################################################
            # Determine the playback speed based on velocities in the demonstration #
            #########################################################################

            max_vel = 0.075 # m/s
            panda_delta_T = 0.01 # 1 ms
            # TODO: this really should probably be switched back to maximum velocity

            if sel_vec[2]==1: # Position control
                max_x = sel_vec[0]*np.average(np.diff(np.array(trajectories[0]),axis=0))
                max_y = sel_vec[1]*np.average(np.diff(np.array(trajectories[1]),axis=0))
                max_z = sel_vec[2]*np.average(np.diff(np.array(trajectories[2]),axis=0))
                average_vel = np.sqrt(np.power(max_x, 2) + np.power(max_y, 2) + np.power(max_z, 2))
            else: # hybrid control
                surfaceModel = PyBSpline.BSplineSurface()
                surfaceModel.loadSurface(surface)

                # x and y are the surface parameters
                # need to get the actual 3D points from the Spline Suface

                points_3d = []

                # 10x downsampling for performance
                downsample = 10
                count = 0
                for traj_x,traj_y in zip(trajectories[0],trajectories[1]):
                    if count%downsample==0:
                        points_3d.append(surfaceModel.calculate_surface_point(traj_x,traj_y)[0])
                    count = count+1

                # 0.1 to cancel out the effect of downsampling
                average_vel = np.average(1.0/(downsample)*np.linalg.norm(np.diff(np.array(points_3d),axis=0),axis=1))


            delta_T = average_vel / max_vel
            num_interp_pts = int(round(delta_T / panda_delta_T))

            print "Average Vel: ", average_vel
            print "Delta T:", delta_T
            print "# interp:", num_interp_pts

            # Don't allow zero
            if num_interp_pts < int(1):
                num_interp_pts = int(1)

            # Actually interpolate the forces
            starting_points, attractor_points, return_forces = dmps[xx].getForces()
            starting_points_r, attractor_points_r, return_forces_r = dmps_reverse[xx].getForces()


            # First write mode to signal new DMP
            surface = segment[3]
            csvfile.write('mode'+','+surface+','+'')
            csvfile.write('\n')
            # Write selection vector
            # TODO: fix this with orientation
            csvfile.write(str(sel_vec[0])+','+str(sel_vec[1])+','+str(sel_vec[2]))
            csvfile.write('\n')
            csvfile.write(str(starting_points[0])+','+str(starting_points[1])+','+str(starting_points[2])+','+str(starting_points[3])+','+str(starting_points[4])+','+str(starting_points[5])+','+str(starting_points[6]))
            csvfile.write('\n')
            csvfile.write(str(attractor_points[0])+','+str(attractor_points[1])+','+str(attractor_points[2])+','+str(attractor_points[3])+','+str(attractor_points[4])+','+str(attractor_points[5])+','+str(attractor_points[6]))
            csvfile.write('\n')

            # Write all of the trajectory stuff
            for ii in range(0,len(return_forces[0])-1):
                # print trajectories[0][ii][0]
                for jj in range(0,num_interp_pts):
                    interp_x = return_forces[0][ii]+(float(jj)/float(num_interp_pts))*(return_forces[0][ii+1]-return_forces[0][ii])
                    interp_y = return_forces[1][ii]+(float(jj)/float(num_interp_pts))*(return_forces[1][ii+1]-return_forces[1][ii])
                    interp_z = return_forces[2][ii]+(float(jj)/float(num_interp_pts))*(return_forces[2][ii+1]-return_forces[2][ii])

                    # TODO: This should be switched to SLERP for Orientation
                    interp_qx = return_forces[3][ii]+(float(jj)/float(num_interp_pts))*(return_forces[3][ii + 1] - return_forces[3][ii])
                    interp_qy = return_forces[4][ii]+(float(jj)/float(num_interp_pts))*(return_forces[4][ii + 1] - return_forces[4][ii])
                    interp_qz = return_forces[5][ii]+(float(jj)/float(num_interp_pts))*(return_forces[5][ii + 1] - return_forces[5][ii])
                    interp_qw = return_forces[6][ii]+(float(jj)/float(num_interp_pts))*(return_forces[6][ii + 1] - return_forces[6][ii])
                    csvfile.write(str(interp_x)+','+str(interp_y)+','+str(interp_z)+','+str(interp_qx)+','+str(interp_qy)+','+str(interp_qz)+','+str(interp_qw))
                    csvfile.write('\n')

            # Write the reverse
            csvfile.write('reverse' + ',' + '' + ',' + '')
            csvfile.write('\n')
            for ii in range(0,len(return_forces[0])-1):
                # print trajectories[0][ii][0]
                for jj in range(0,num_interp_pts):
                    interp_x = return_forces_r[0][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[0][ii+1]-return_forces_r[0][ii])
                    interp_y = return_forces_r[1][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[1][ii+1]-return_forces_r[1][ii])
                    interp_z = return_forces_r[2][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[2][ii+1]-return_forces_r[2][ii])

                    # TODO: This should be switched to SLERP for Orientation
                    interp_qx = return_forces_r[3][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[3][ii + 1] - return_forces_r[3][ii])
                    interp_qy = return_forces_r[4][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[4][ii + 1] - return_forces_r[4][ii])
                    interp_qz = return_forces_r[5][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[5][ii + 1] - return_forces_r[5][ii])
                    interp_qw = return_forces_r[6][ii]+(float(jj)/float(num_interp_pts))*(return_forces_r[6][ii + 1] - return_forces_r[6][ii])
                    csvfile.write(str(interp_x)+','+str(interp_y)+','+str(interp_z)+','+str(interp_qx)+','+str(interp_qy)+','+str(interp_qz)+','+str(interp_qw))
                    csvfile.write('\n')

            # Write the variance
            csvfile.write('variance' + ',' + '' + ',' + '')
            csvfile.write('\n')
            for ii in range(0,len(return_forces[0])-1):
                # print trajectories[0][ii][0]
                for jj in range(0,num_interp_pts):

                    if(variances is not ""):
                        # Presegmented comes with set level of variances. Otherwise, use based on the data
                        csvfile.write(str(variances[0])+','+str(variances[1])+','+str(variances[2]))
                        csvfile.write('\n')

                    else:
                        # LFD gets variance based on the data
                        var_x = variance_per_segment[xx][0][ii] + (float(jj) / float(num_interp_pts)) * (
                                    variance_per_segment[0][0][ii] - variance_per_segment[0][0][ii])
                        var_y = variance_per_segment[xx][1][ii] + (float(jj) / float(num_interp_pts)) * (
                                    variance_per_segment[0][1][ii] - variance_per_segment[0][1][ii])
                        var_z = variance_per_segment[xx][2][ii] + (float(jj) / float(num_interp_pts)) * (
                                    variance_per_segment[0][2][ii] - variance_per_segment[0][2][ii])
                        csvfile.write(str(var_x) + ',' + str(var_y) + ',' + str(var_z))
                        csvfile.write('\n')


def main():
    global path0, path1, path2, path3, path4, pathDMP, forceDMP
    rospy.init_node('dmppathlistener', anonymous=True)
    rospy.Subscriber("/dmp/filepubsegmented", String, loadPresegmented)
    rospy.spin()

if __name__ == "__main__":
    main()




