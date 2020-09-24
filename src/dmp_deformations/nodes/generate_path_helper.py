#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import PyBSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import rospkg


def printCircularSection(csvfile, starting_point, ending_point, num_pts, radius, frequency):
    # Set up interpolation of the quaternions
    key_inds = [0, num_pts - 1];
    key_rots = R.from_quat(np.array([[starting_point[3], starting_point[4], starting_point[5], starting_point[6]],
                                     [ending_point[3], ending_point[4], ending_point[5], ending_point[6]]]))


    pts = []

    slerp = Slerp(key_inds, key_rots)

    for ii in range(0, num_pts):
        # Position
        interp_x = starting_point[0] + (ending_point[0] - starting_point[0]) * (float(ii) / float(num_pts - 1)) + ((float(ii)/num_pts<0.1)*(float(ii)/(0.1*num_pts))+float((float(ii)/num_pts>=0.1))+(float(ii)/num_pts>0.9)*(-10.0*(float(ii)/num_pts-0.9)))*radius*np.sin(frequency*float(ii)/num_pts)
        interp_y = starting_point[1] + (ending_point[1] - starting_point[1]) * (float(ii) / float(num_pts - 1)) + ((float(ii)/num_pts<0.1)*(float(ii)/(0.1*num_pts))+float((float(ii)/num_pts>=0.1))+(float(ii)/num_pts>0.9)*(-10.0*(float(ii)/num_pts-0.9)))*radius*np.cos(frequency*float(ii)/num_pts)
        interp_z = starting_point[2] + (ending_point[2] - starting_point[2]) * (float(ii) / float(num_pts - 1))

        # Orientation
        temp_q = slerp([ii])
        interp_qx = temp_q.as_quat()[0][0]
        interp_qy = temp_q.as_quat()[0][1]
        interp_qz = temp_q.as_quat()[0][2]
        interp_qw = temp_q.as_quat()[0][3]

        # Forces
        interp_fx = starting_point[7] + (ending_point[7] - starting_point[7]) * (float(ii) / float(num_pts - 1))
        interp_fy = starting_point[8] + (ending_point[8] - starting_point[8]) * (float(ii) / float(num_pts - 1))
        interp_fz = starting_point[9] + (ending_point[9] - starting_point[9]) * (float(ii) / float(num_pts - 1))

        # Torques
        interp_tx = starting_point[10] + (ending_point[10] - starting_point[10]) * (float(ii) / float(num_pts - 1))
        interp_ty = starting_point[11] + (ending_point[11] - starting_point[11]) * (float(ii) / float(num_pts - 1))
        interp_tz = starting_point[12] + (ending_point[12] - starting_point[12]) * (float(ii) / float(num_pts - 1))

        csvfile.write(str(interp_x) + ',' + str(interp_y) + ',' + str(interp_z) + ','
                      + str(interp_qx) + ',' + str(interp_qy) + ',' + str(interp_qz) + ',' + str(interp_qw) + ','
                      + str(interp_fx) + ',' + str(interp_fy) + ',' + str(interp_fz) + ','
                      + str(interp_tx) + ',' + str(interp_ty) + ',' + str(interp_tz))
        csvfile.write('\n')

        #print(interp_x,interp_y)
        pts.append((interp_x, interp_y))

    pts = np.array(pts)
    plt.plot(pts[:,0],pts[:,1])
    plt.axis("Equal")
    plt.show()


def printHoleSpiral(csvfile, starting_point, num_pts, alpha_a, alpha_b, frequency):
    # Set up interpolation of the quaternions
    key_inds = [0, num_pts - 1];
    key_rots = R.from_quat(np.array([[starting_point[3], starting_point[4], starting_point[5], starting_point[6]],
                                     [starting_point[3], starting_point[4], starting_point[5], starting_point[6]]]))

    pts = []

    slerp = Slerp(key_inds, key_rots)

    theta = 0

    for ii in range(0, num_pts):
        # Position
        theta = theta+(1-(float(ii)/float(num_pts)))
        interp_x = starting_point[0] + alpha_a*np.sin(frequency/5*theta/num_pts)
        interp_y = starting_point[1] + alpha_b*np.sin(frequency*theta/num_pts)
        interp_z = starting_point[2]

        # Orientation
        temp_q = slerp([ii])
        interp_qx = temp_q.as_quat()[0][0]
        interp_qy = temp_q.as_quat()[0][1]
        interp_qz = temp_q.as_quat()[0][2]
        interp_qw = temp_q.as_quat()[0][3]

        # Forces
        interp_fx = starting_point[7]
        interp_fy = starting_point[8]
        interp_fz = starting_point[9]

        # Torques
        interp_tx = starting_point[10]
        interp_ty = starting_point[11]
        interp_tz = starting_point[12]

        csvfile.write(str(interp_x) + ',' + str(interp_y) + ',' + str(interp_z) + ','
                      + str(interp_qx) + ',' + str(interp_qy) + ',' + str(interp_qz) + ',' + str(interp_qw) + ','
                      + str(interp_fx) + ',' + str(interp_fy) + ',' + str(interp_fz) + ','
                      + str(interp_tx) + ',' + str(interp_ty) + ',' + str(interp_tz))
        csvfile.write('\n')

        #print(interp_x,interp_y)
        pts.append((interp_x, interp_y))

    pts = np.array(pts)
    plt.plot(pts[:,0],pts[:,1])
    plt.axis("Equal")
    plt.show()

def printPathSection(csvfile,starting_point,ending_point,num_pts):

    # Set up interpolation of the quaternions
    key_inds = [0, num_pts-1];
    key_rots = R.from_quat(np.array([[starting_point[3], starting_point[4], starting_point[5], starting_point[6]],[ending_point[3], ending_point[4], ending_point[5], ending_point[6]]]))
    slerp = Slerp(key_inds, key_rots)

    for ii in range(0,num_pts):
        # Position
        interp_x = starting_point[0]+(ending_point[0]-starting_point[0])*(float(ii)/float(num_pts-1))
        interp_y = starting_point[1]+(ending_point[1]-starting_point[1])*(float(ii)/float(num_pts-1))
        interp_z = starting_point[2]+(ending_point[2]-starting_point[2])*(float(ii)/float(num_pts-1))

        # Orientation
        temp_q = slerp([ii])
        interp_qx = temp_q.as_quat()[0][0]
        interp_qy = temp_q.as_quat()[0][1]
        interp_qz = temp_q.as_quat()[0][2]
        interp_qw = temp_q.as_quat()[0][3]

        # Forces
        interp_fx = starting_point[7]+(ending_point[7]-starting_point[7])*(float(ii)/float(num_pts-1))
        interp_fy = starting_point[8]+(ending_point[8]-starting_point[8])*(float(ii)/float(num_pts-1))
        interp_fz = starting_point[9]+(ending_point[9]-starting_point[9])*(float(ii)/float(num_pts-1))

        # Torques
        interp_tx = starting_point[10] + (ending_point[10] - starting_point[10]) * (float(ii) / float(num_pts - 1))
        interp_ty = starting_point[11] + (ending_point[11] - starting_point[11]) * (float(ii) / float(num_pts - 1))
        interp_tz = starting_point[12] + (ending_point[12] - starting_point[12]) * (float(ii) / float(num_pts - 1))
       
        csvfile.write(str(interp_x)+','+str(interp_y)+','+str(interp_z)+','
                      + str(interp_qx) + ',' + str(interp_qy) + ',' + str(interp_qz) + ',' + str(interp_qw) + ','
                      + str(interp_fx)+','+str(interp_fy)+','+str(interp_fz) + ','
                      + str(interp_tx) + ',' + str(interp_ty) + ',' + str(interp_tz))
        csvfile.write('\n')


def calculateQuaternion(normal_vector, x, y):
    R_new = np.eye(3)
    R_new[:,0]=x.reshape((3,))
    R_new[:,1]=y.reshape((3,))
    R_new[:,2]=normal_vector.reshape((3,))

    r = R.from_dcm(R_new)
    q_new = r.as_quat() # x,y,z,w
    qx = q_new[0]
    qy = q_new[1]
    qz = q_new[2]
    qw = q_new[3]

    return qx,qy,qz,qw

def layup1():
    # Number of points per section
    num_pts = 100
    rospack = rospkg.RosPack()
    path_devel = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations/"
    ##########################################################
    #   Layup 1 TASK #########################################
    ##########################################################
    with open(path_devel+'layup1_execution.csv', 'w') as csvfile:

        surfaceModel = PyBSpline.BSplineSurface()
        surfaceModel.loadSurface("layup1")

        csvfile.write(",layup1,,,layup1,")
        csvfile.write('\n')
        csvfile.write("0,100,200,300,400,500")
        csvfile.write('\n')
        csvfile.write("1,0,1,1,0,1")
        csvfile.write('\n')
        # csvfile.write("2 2 2,3 10 150,2 2 2")
        csvfile.write("2 2 2,10 10 150,2 2 2,2 2 2,10 10 150,2 2 2")
        csvfile.write('\n')

        surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.8, 0.8)

        starting_vel = r_u * -1.0 + r_v * 0.0
        starting_vel = starting_vel / np.linalg.norm(starting_vel)
        starting_y = np.cross(normal_start, starting_vel)
        qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)

        above_surf = surface_start + 0.005*normal_start


        printPathSection(csvfile, np.array(
            [0.50, 0.0, 0.25, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array(
                             [above_surf[0], above_surf[1], above_surf[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0,
                              0.0, 0.0, 0.0]), num_pts)

        # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
        printPathSection(csvfile, np.array([0.8, 0.8, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array([0.2, 0.8, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)

        surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.2, 0.8)
        ending_vel = r_u * -1.0 + r_v* 0.0
        ending_vel = ending_vel / np.linalg.norm(ending_vel)
        ending_y = np.cross(normal_end, ending_vel)
        qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)

        printPathSection(csvfile, np.array(
            [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array([0.50, 0.0, 0.25, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)

        surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.8, 0.8)

        starting_vel = r_u * 0.0 + r_v * -1.0
        starting_vel = starting_vel / np.linalg.norm(starting_vel)
        starting_y = np.cross(normal_start, starting_vel)
        qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)

        above_surf = surface_start + 0.005 * normal_start

        printPathSection(csvfile, np.array(
            [0.50, 0.0, 0.25, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array(
                             [above_surf[0], above_surf[1], above_surf[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0,
                              0.0, 0.0, 0.0]), num_pts)

        # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
        printPathSection(csvfile, np.array([0.8, 0.8, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array([0.8, 0.2, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)

        surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.8, 0.2)
        ending_vel = r_u * 0.0 + r_v* -1.0
        ending_vel = ending_vel / np.linalg.norm(ending_vel)
        ending_y = np.cross(normal_end, ending_vel)
        qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)

        printPathSection(csvfile, np.array(
            [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array([0.50, 0.0, 0.25, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)
def layup2():
    # Number of points per section
    num_pts = 100
    rospack = rospkg.RosPack()
    path_devel = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations/"
    ##########################################################
    #   Layup 2 TASK #########################################
    ##########################################################
    with open(path_devel + 'layup2_execution.csv', 'w') as csvfile:
        surfaceModel = PyBSpline.BSplineSurface()
        surfaceModel.loadSurface("layup2")

        csvfile.write(
            ",layup2,,,,layup2,,,,layup2,,,,layup2,,,,layup2,,,,layup2,,,,layup2,,,,layup2,,,,layup2,,,,layup2,,")
        csvfile.write('\n')
        csvfile.write(",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,") # preaction
        csvfile.write('\n')
        csvfile.write(
            "0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,3300,3400,3500,3600,3700,3800,3900")
        csvfile.write('\n')
        csvfile.write("1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1")
        csvfile.write('\n')
        # csvfile.write("2 2 2,3 10 150,2 2 2")
        csvfile.write(
            "2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2,2 2 2,2 2 2,10 10 150,2 2 2")
        csvfile.write('\n')
        csvfile.write("0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.3,0.3,0.3,0.12,0.2,0.1")
        csvfile.write('\n')
        homing_point = np.array([0.6, -0.3, 0.20])
        homing_point_2 = np.array([0.52, 0.1, 0.20])
        force = -5.0

        for ii in np.arange(0.95, 0.004, -0.1):
            print("ii", ii)
            # START OF PASS
            surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(ii, 0.98)
            print('SS:', surface_start)

            starting_vel = r_u * 0.0 + r_v * -1.0
            starting_vel = starting_vel / np.linalg.norm(starting_vel)
            starting_y = np.cross(normal_start, starting_vel)
            qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)

            above_surf = surface_start + 0.03 * normal_start

            printPathSection(csvfile, np.array(
                [homing_point[0], homing_point[1], homing_point[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [above_surf[0], above_surf[1], above_surf[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)

            # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
            printPathSection(csvfile, np.array([ii, 0.98, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force, 0.0, 0.0, 0.0]),
                             np.array([ii, 0.02, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force, 0.0, 0.0, 0.0]), num_pts)

            surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(ii, 0.02)
            ending_vel = r_u * 0.0 + r_v * -1.0
            ending_vel = ending_vel / np.linalg.norm(ending_vel)
            ending_y = np.cross(normal_end, ending_vel)
            qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)

            printPathSection(csvfile, np.array(
                [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -5.0, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [homing_point_2[0], homing_point_2[1], homing_point_2[2], qx_s, qy_s, qz_s, qw_s, 0.0,
                                  0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)

            printPathSection(csvfile, np.array(
                [homing_point_2[0], homing_point_2[1], homing_point_2[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0,
                 0.0,
                 0.0]),
                             np.array(
                                 [homing_point[0], homing_point[1], homing_point[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0,
                                  -5.0, 0.0,
                                  0.0, 0.0]), num_pts)
            # END OF PASS


def cowling4():
    # Number of points per section
    num_pts = 100
    rospack = rospkg.RosPack()
    path_devel = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations/"
    ##########################################################
    #   Layup 2 TASK #########################################
    ##########################################################
    with open(path_devel + 'cowling4_execution.csv', 'w') as csvfile:
        surfaceModel = PyBSpline.BSplineSurface()
        surfaceModel.loadSurface("cowling4")

        csvfile.write(",cowling4,,")
        csvfile.write('\n')
        csvfile.write(",,,") #preaction
        csvfile.write('\n')
        csvfile.write("0,100,200")
        csvfile.write('\n')
        csvfile.write("1,0,1")
        csvfile.write('\n')
        csvfile.write("2 2 2,5 5 500,2 2 2")
        csvfile.write('\n')

        homing_point = np.array([0.45, -0.3, 0.20])
        q_straight = np.array([0.7071068, 0.0, 0.0, 0.7071068])
        force = -10.0

        # START OF PASS
        surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.2,0.2)

        starting_vel = r_u * 0.0 + r_v * 1.0
        starting_vel = starting_vel / np.linalg.norm(starting_vel)
        starting_y = np.cross(normal_start, starting_vel)
        qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)

        above_surf = surface_start + 0.03 * normal_start

        printPathSection(csvfile, np.array(
            [homing_point[0], homing_point[1], homing_point[2], q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force, 0.0, 0.0,
             0.0]),
                         np.array(
                             [above_surf[0], above_surf[1], above_surf[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force,
                              0.0, 0.0, 0.0]), num_pts)

        # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
        printCircularSection(csvfile, np.array([0.2, 0.2, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force, 0.0, 0.0, 0.0]),
                         np.array([0.2, 0.8, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force, 0.0, 0.0, 0.0]), num_pts,0.035,-50)

        surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.2, 0.8)
        ending_vel = r_u * 0.0 + r_v * 1.0
        ending_vel = ending_vel / np.linalg.norm(ending_vel)
        ending_y = np.cross(normal_end, ending_vel)
        qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)

        printPathSection(csvfile, np.array(
            [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -5.0, 0.0, 0.0,
             0.0]),
                         np.array(
                             [homing_point[0], homing_point[1], homing_point[2], qx_s, qy_s, qz_s, qw_s, 0.0,
                              0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)


def fastenerInsertion():
    # Number of points per section
    num_pts = 100
    rospack = rospkg.RosPack()
    path_devel = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations/"
    ##########################################################
    #   Layup 2 TASK #########################################
    ##########################################################
    with open(path_devel + 'fastener1_execution.csv', 'w') as csvfile:
        surfaceModel = PyBSpline.BSplineSurface()
        surfaceModel.loadSurface("fastener1")

        csvfile.write(",,,,,fastener1,,,,,,,fastener1,,,,,,,fastener1,")
        csvfile.write('\n')
        csvfile.write("release,,grasp,,,peginhole,,release,,grasp,,,peginhole,,release,,grasp,,,peginhole,,") # preaction/conditional logic fxn
        csvfile.write('\n')
        csvfile.write("0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000")
        csvfile.write('\n')
        csvfile.write("1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,0,1")
        csvfile.write('\n')
        csvfile.write("2 2 2,2 2 2,2 2 2,2 2 2,0.5 0.5 0.5,3 30 150,2 2 2,2 2 2,2 2 2,2 2 2,2 2 2,0.5 0.5 0.5,3 30 150,2 2 2,2 2 2,2 2 2,2 2 2,2 2 2,0.5 0.5 0.5,3 30 150,2 2 2")
        csvfile.write('\n')
        csvfile.write("0.2,0.075,0.2,0.2,0.02,0.01,0.1,0.2,0.075,0.2,0.2,0.02,0.01,0.1,0.2,0.075,0.2,0.2,0.02,0.01,0.1")
        csvfile.write('\n')

        # Apply a rotation and a translation to all of the points
        R = np.array([[0.99978826, 0.00928849, -0.01836173],
                      [-0.00907831, 0.9998927, 0.01149706],
                      [0.01846655, -0.01132793, 0.9997653]])


        t_addtl = np.matmul(R, np.array([0.025,0.0, 0.0]).reshape((3, 1))).reshape((3,))

        holder_location = np.array([0.52434668, 0.21501005, 0.02388816])

        holder_locations = []
        holder_locations.append(holder_location+t_addtl)
        holder_locations.append(holder_location)
        holder_locations.append(holder_location-t_addtl)

        cowling_location = np.array([ 0.5085376 , -0.06548236,  0.14053339])

        cowling_locations = []
        cowling_locations.append((0.75,0.5))
        cowling_locations.append((0.38,0.5))
        cowling_locations.append((0.15,0.5))

        for ii in range(0,3):

            holder_location = holder_locations[ii]

            # Hybrid
            surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(cowling_locations[ii][0],cowling_locations[ii][1])
            starting_vel = r_u * 0.0 + r_v * 1.0
            starting_vel = starting_vel / np.linalg.norm(starting_vel)
            starting_y = np.cross(normal_start, starting_vel)
            qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)
            above_surf = surface_start + 0.073*normal_start

            print("NORMAL:",normal_start)

            print("AS:",above_surf)
            print("SS:",surface_start)

            homing_point = np.array([0.45, 0.0, 0.3])
            q_straight = np.array([0.0, 0.0, 0.0, 1.0])
            q_sideways = np.array([0.0, 0.0, 0.7071068, 0.7071068])
            force = -5.0

            printPathSection(csvfile, np.array(
                [homing_point[0], homing_point[1], homing_point[2], q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [holder_location[0], holder_location[1], holder_location[2]+0.15, q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)

            printPathSection(csvfile, np.array(
                [holder_location[0], holder_location[1], holder_location[2]+0.13, q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [holder_location[0], holder_location[1], holder_location[2]+0.055, q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)

            printPathSection(csvfile, np.array(
                [holder_location[0], holder_location[1], holder_location[2]+0.055, q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [holder_location[0], holder_location[1], holder_location[2]+0.3, q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)

            printPathSection(csvfile, np.array(
                [holder_location[0], holder_location[1], holder_location[2]+0.3, q_straight[0], q_straight[1], q_straight[2], q_straight[3], 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [above_surf[0], above_surf[1], above_surf[2]+0.05, q_sideways[0], q_sideways[1], q_sideways[2], q_sideways[3], 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)

            printPathSection(csvfile, np.array(
                [above_surf[0], above_surf[1], above_surf[2]+0.05, q_straight[0], q_sideways[1], q_sideways[2], q_sideways[3], 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [above_surf[0], above_surf[1], above_surf[2], q_sideways[0], q_sideways[1], q_sideways[2], q_sideways[3], 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)

            printHoleSpiral(csvfile, np.array([cowling_locations[ii][0], cowling_locations[ii][1], 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, force, 0.0, 0.0, 0.0]),num_pts,0.015,0.2,15)

            printPathSection(csvfile, np.array(
                [above_surf[0], above_surf[1], above_surf[2]+0.15, q_sideways[0], q_sideways[1], q_sideways[2], q_sideways[3], 0.0, 0.0, force, 0.0, 0.0,
                 0.0]),
                             np.array(
                                 [homing_point[0], homing_point[1], homing_point[2], q_sideways[0], q_sideways[1], q_sideways[2], q_sideways[3], 0.0, 0.0, force,
                                  0.0, 0.0, 0.0]), num_pts)


def main():
    fastenerInsertion()

if __name__ == "__main__":
    main()




