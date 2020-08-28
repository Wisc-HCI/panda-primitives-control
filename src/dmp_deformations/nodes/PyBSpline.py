""" Uniform B-spline surfaces
    based on user input of control
    points and curve order

 Created: 05/07/2020
"""

__author__ = "Mike Hagenow"

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import math
import csv
from scipy.optimize import minimize, Bounds
import rospkg


def exp_map_np(w):
    w = np.array(w)
    w = w.reshape(3, 1)
    theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
    w = w / theta
    w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))


class BSplineSurface:

    def initialize(self,k=3,control_pts=np.zeros((1,1,3)),u_dir=None, v_dir=None):
        # Parameters for building B-Spline Surface
        self.k = k # order of the B-spline
        self.controls_pts = control_pts # n control points by 3 (x,y,z)
        num_control_pts = np.shape(control_pts)[0:2]
        if num_control_pts[0] <= k or num_control_pts[1]<=k:
            raise Exception("Parameter","Not enough control pts for curve degree")
        self.m = num_control_pts[0]-1 # number of control points in u-direction 0-> m
        self.n = num_control_pts[1]-1  # number of control points in v-direction 0 -> n

        # Need to add the first and last value k times (total of (m-k+2) = ((m+k+2) - 2k)
        self.knots_u = np.concatenate([np.zeros((self.k,)), np.linspace(0, 1, (self.m-self.k+2)),
                                       np.ones((self.k,))])  # number uniform knots in u-direction
        self.knots_v = np.concatenate([np.zeros((self.k,)), np.linspace(0, 1, (self.n-self.k+2)),
                                       np.ones((self.k,))])  # number uniform knots in v-direction


        # u and v directions to be used for input mapping
        if u_dir is not None and v_dir is not None:
            self.u_dir = u_dir
            self.v_dir = v_dir
        else:
            self.u_dir = np.array([1,0,0])
            self.v_dir = np.array([0,1,0])

        #self.u_dir, self.v_dir = self.calculate_planar_directions()

    def exp_map_np(self,w):
        w = np.array(w)
        w = w.reshape(3, )
        theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
        w = w / theta
        w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))

    # x are the parameters, stored as wx,wy,d
    # y are the points in the point cloud
    def obj_plane_dist(self,x, pts):
        sum = 0.0
        for pt in pts:
            # equation for the plane - sum of residual is used as the LSQ error
            vec_to_plane_center = np.transpose(
                np.matmul(exp_map_np([x[0], x[1], 0.0]), np.array([0.0, 0.0, x[2]]).reshape((3, 1))) - pt.reshape(
                    (3, 1)))
            sum += np.abs(np.matmul(vec_to_plane_center, np.matmul(exp_map_np([x[0], x[1], 0.0]),
                                                                   np.array([0.0, 0.0, 1.0]).reshape((3, 1))))[0][0])
        # print x[0], x[1], x[2], sum
        return sum

    def findPlane(self,pts):
        best_opt = np.inf
        for ii in range(0, 5):
            wx = np.random.rand() * np.pi / 2
            wy = np.random.rand() * np.pi / 2
            d = 1.0 * (np.random.rand() - 0.5)

            x0 = np.array([wx, wy, d])
            res = minimize(self.obj_plane_dist, x0, method='Nelder-Mead',
                           options={'disp': False}, bounds=None, args=(pts))

            if res.fun < best_opt:
                best_opt = res.fun
                wx_best = res.x[0]
                wy_best = res.x[1]
                d_best = res.x[2]

        return wx_best, wy_best, d_best

    def calculate_surface_point(self,u,v):
        # Calculate all of the basis functions for the given u,v
        # using recursive formulation
        r = np.array([0.0, 0.0, 0.0])

        ########################################
        # Calculate the interpolated point     #
        ########################################

        # Calculate the sum of basis functions for the point interpolation
        for ii in range(0,self.m+1):
            for jj in range(0,self.n+1):
                r+=self.getN(ii,self.k,u,self.knots_u)*self.getN(jj,self.k,v,self.knots_v)*self.controls_pts[ii,jj,:]

        ########################################
        # Calculate the normal                 #
        ########################################

        # Partial in the U-direction
        r_u = np.array([0.0, 0.0, 0.0])
        for ii in range(0,self.m):
            for jj in range(0,self.n+1):
                scaling_temp =0.0
                if (self.knots_u[ii+self.k+1]-self.knots_u[ii+1]) != 0:
                    scaling_temp = self.k/(self.knots_u[ii+self.k+1]-self.knots_u[ii+1])
                r_u+=self.getN(ii,self.k-1,u,self.knots_u[1:-1])*self.getN(jj,self.k,v,self.knots_v)*scaling_temp*(self.controls_pts[ii+1,jj,:]-self.controls_pts[ii,jj,:])

        # Partial in the V-direction
        r_v = np.array([0.0, 0.0, 0.0])
        for ii in range(0, self.m+1):
            for jj in range(0, self.n):
                scaling_temp = 0.0
                if (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1]) != 0:
                    scaling_temp = self.k / (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1])
                r_v += self.getN(ii, self.k, u, self.knots_u) * self.getN(jj, self.k-1, v,self.knots_v[1:-1]) * scaling_temp * (self.controls_pts[ii, jj + 1, :] - self.controls_pts[ii, jj, :])


        # Get the surface normal from the cross-product
        r_u_norm = np.divide(r_u,np.linalg.norm(r_u))
        r_v_norm = np.divide(r_v,np.linalg.norm(r_v))
        n_hat = np.cross(r_u_norm,r_v_norm)

        # return the calculated point
        return r, n_hat, r_u_norm, r_v_norm

    def getN(self,i,p,x,t):
        # Recursive function that calculates the basis
        # function value using De Boor's
        # (simple version of the algorithm with nan checking)

        if p is 0: # 0-th level is a boolean of sorts
            if x>=t[i] and x<t[i+1]: # Firing condition
                return 1.0
            else: # No Fire Condition
                return 0.0

        else: # other levels are recursions based on lower levels
            part_a = 0.0
            part_b = 0.0

            # Check for the two potential divide by zero cases (should be 0)
            if((t[i+p]-t[i]) != 0):
                part_a = (x-t[i])/(t[i+p]-t[i])*self.getN(i,p-1,x,t)
            if (t[i+p+1]-t[i+1]) !=0:
                part_b = (t[i+p+1]-x)/(t[i+p+1]-t[i+1])*self.getN(i+1,p-1,x,t)
            return part_a + part_b

    def obj_closest_surface(self, x, surface, xf, yf, zf):
        r, n_hat, r_u_norm, r_v_norm = surface.calculate_surface_point(x[0], x[1])
        return (r[0] - xf) * (r[0] - xf) + (r[1] - yf) * (r[1] - yf) + (r[2] - zf) * (r[2] - zf)

    def getClosestParams(self, x, y, z, u, v):
        x0 = np.array([u, v])
        bounds = Bounds([0.0, 0.0], [1.0, 1.0])
        res = minimize(self.obj_closest_surface, x0, method='L-BFGS-B',
                       options={'disp': False}, bounds=bounds, args=(self, x, y, z))
        return res.x[0], res.x[1]

    def writeSurface(self,filename):
        rospack = rospkg.RosPack()
        path_devel = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations/"
        with open(path_devel+filename+'.csv', 'w') as csvfile:
            # Only parameters needed are degree and control points
            csvfile.write(str(self.k) + ',' + str(self.m) + ',' + str(self.n))
            csvfile.write('\n')

            # Load all of the control points
            for ii in range(0, self.m + 1):
                for jj in range(0, self.n + 1):
                    csvfile.write(str(self.controls_pts[ii,jj,0])+' '+str(self.controls_pts[ii,jj,1])+' '+
                                  str(self.controls_pts[ii,jj,2])+',')
                csvfile.write('\n')

            # write u and v directions
            csvfile.write(str(self.u_dir[0]) + ',' + str(self.u_dir[1]) + ',' +
            str(self.u_dir[2]) + ',')
            csvfile.write('\n')
            csvfile.write(str(self.v_dir[0]) + ',' + str(self.v_dir[1]) + ',' +
            str(self.v_dir[2]) + ',')
            csvfile.write('\n')

    def loadSurface(self, filename):
        rospack = rospkg.RosPack()
        path_devel = rospack.get_path('dmp_deformations') + "/../../devel/lib/dmp_deformations/"
        with open(path_devel + filename + '.csv') as csvfile:
            surfacereader = csv.reader(csvfile,delimiter=',')

            # Load the parameters and control points
            row = 0
            for row_temp in surfacereader:
                if row==0:
                    k = int(row_temp[0])
                    num_control_u = int(row_temp[1])
                    num_control_v = int(row_temp[2])
                    control_pts = np.zeros((num_control_u+1,num_control_v+1,3))
                else:
                    col=0
                    if row<=(num_control_u+1): # don't include u and v directions at the end
                        for ii in range(0,num_control_v+1):
                                point = row_temp[ii]
                                values = point.split(' ')
                                # one extra row for parameters
                                control_pts[row-1,col,0]=float(values[0])
                                control_pts[row-1,col,1]=float(values[1])
                                control_pts[row-1,col,2]=float(values[2])
                                col+=1
                row+=1

            # initialize the B-Spline
            self.initialize(k=k,control_pts=control_pts)


def createCurved():
    x = np.linspace(0.15, -0.35, 25)
    y = np.linspace(0.05, -0.25, 25)
    xv, yv = np.meshgrid(x, y)
    z = 0.865+0.05*np.sin(15*(0.15-xv))+0.06*np.sin((np.pi/0.3)*(0.05-yv)-np.pi/2)

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))
    print(control_pts[0,24,:])

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    a,b,c,d = test_curve.calculate_surface_point(0.2, 0.8)
    print("PT:",a)

    # Evaluate a new point
    # eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    # print "NHAT: ", n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(a[0],a[1],a[2],color='red')
    # ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2], n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    # ax.set_xlim3d(0, 1)
    # ax.set_ylim3d(0, 1)
    # ax.set_zlim3d(0, 1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('curved')

def createAngledPlane():
    a = np.linspace(-0.4, 0.4, 25)
    b = np.linspace(-0.4, 0.4, 25)
    control_pts = np.zeros((25,25,3))
    for aa in range(len(a)):
        for bb in range(len(b)):
            control_pts[aa,bb,:]=np.matmul(exp_map_np([1.37542454, 0.37261167, 0.0]),np.array([a[aa],b[bb],-0.2968528]))

    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))
    print(control_pts[0, 24, :])

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    a, b, c, d = test_curve.calculate_surface_point(0.3, 0.6)
    print("PT:", a)

    # Evaluate a new point
    # eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    # print "NHAT: ", n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(control_pts[:,:,0], control_pts[:,:,1], control_pts[:,:,2], color='blue')
    ax.scatter(.0080, .34688, .215, color='red')

    ut,vt = test_curve.getClosestParams(.0080, .34688, .215,0.5,0.5)
    print("UV:",ut," ",vt)

    # ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2], n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    # ax.set_xlim3d(0, 1)
    # ax.set_ylim3d(0, 1)
    # ax.set_zlim3d(0, 1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('plane')

def createTable():
    x = np.linspace(0.32, -0.68, 25)
    y = np.linspace(0.18, -0.32, 25)
    xv, yv = np.meshgrid(x, y)
    z = 0.795+0.0*xv+0.0*yv

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))
    print(control_pts[0,24,:])

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    a,b,c,d = test_curve.calculate_surface_point(0.3, 0.6)
    print("PT:",a)

    # Evaluate a new point
    # eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    # print "NHAT: ", n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(a[0],a[1],a[2],color='red')
    # ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2], n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    # ax.set_xlim3d(0, 1)
    # ax.set_ylim3d(0, 1)
    # ax.set_zlim3d(0, 1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('table')

def createBoeing():
    x = np.linspace(0.0, 0.2032, 30)
    y = np.linspace(0.0, 0.2032, 30)
    xv, yv = np.meshgrid(x, y)
    z = 0.0762+0.0*xv+0.0*yv
    z[14:,14:]=0.0762/2

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))
    print(control_pts[0,24,:])

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    a,b,c,d = test_curve.calculate_surface_point(0.3, 0.6)
    print("PT:",a)

    # Evaluate a new point
    # eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    # print "NHAT: ", n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(a[0],a[1],a[2],color='red')
    # ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2], n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    # ax.set_xlim3d(0, 1)
    # ax.set_ylim3d(0, 1)
    # ax.set_zlim3d(0, 1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('boeinglayup')

def testing():
    # Create surface test data
    x = np.linspace(0, 1, 25)
    y = np.linspace(0, 1, 25)
    xv, yv = np.meshgrid(x, y)
    z = 0.1 * np.abs(np.sin(xv * np.pi*1)) + 0.2 * np.sin(yv * np.pi*2)+0.2

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    # Evaluate a new point
    eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    print "NHAT: ",n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2],n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    ax.set_xlim3d(0,1)
    ax.set_ylim3d(0,1)
    ax.set_zlim3d(0,1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('test')

def unit_testing():
    test_curve = BSplineSurface()
    print(test_curve.getN(0,2,0.25,np.array([0.0, 0.0, 0.0, float(1.0/3), float(2.0/3), 1.0, 1.0, 1.0])))

def curvedLoad():
    test = BSplineSurface()
    test.loadSurface("curved")
    print(test.calculate_surface_point(0.1,0.1))

if __name__ == "__main__":
    testing()




