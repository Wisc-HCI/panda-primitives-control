""" Calculates DMP from demonstration data

 Inspired by Calinon et al.
 See pbdlib
 Created: 02/20/2020
"""

__author__ = "Mike Hagenow"

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import mayavi.mlab as mlab
from scipy.interpolate import interp1d
from GMM import GMM

class DMP:
    # Parameters for building DMP
    num_points = 200 # number of points in learned DMP
    dt = 0.01 # sample time of DMP
    num_variables = 3 # number of independent DMPs
    gmm_states = 50
    s_alpha = 3/(num_points*dt);  # decay factor for forcing term - 3 tau i.e., 95%
    number_demos = 0

    k = 50
    b = np.power((2*k),(0.5)) # ideal underdamped damping ratio

    # Internal Variables
   
    tbGMM = None

    def __init__(self):
        self.attractor_points=[]
        self.starting_points=[]
        self.s = np.exp(-self.s_alpha * np.arange(0,self.num_points)*self.dt) # decay function
        self.desired_forcing = []
        self.weighting_fxns=[]
        self.DMP_forcing=[]

        # print "Initializing"



    # Expects the following format for demonstration data
    # it is a list where each entry is a separate demonstration.
    # Each entry will be mxn where m is the length
    # of the data and n is the number of variables
    # for one variable, the shape must be e.g., (200,1)
    # as opposed to (200,)
    def inputData(self,demonstration_data):
        # Clear data if non-empty
        self.kinematic_data=[]
        self.desired_forcing=[]

        # TODO: fix this, hack to make it work for not exacly 200 points!!!
        self.s = np.exp(-self.s_alpha * np.arange(0, self.num_points) * self.dt)  # decay function

        if self.num_points<200:
            self.gmm_states = self.num_points/4

        # demonstration_data is a list of demonstrations
        self.number_demos = len(demonstration_data)

        # get number of variables
        self.num_variables = np.shape(demonstration_data[0])[1] # number of columns

        # attractor point is the median last value from the demonstrations

        self.starting_points = np.zeros((self.num_variables,))
        self.attractor_points = np.zeros((self.num_variables,))


        for jj in range(0,self.num_variables):
            # just use the actual point for now
            ending_points = [demo[-1:,jj] for demo in demonstration_data]
            starting_points = [demo[0:1, jj] for demo in demonstration_data]
            self.attractor_points[jj]=np.median(ending_points)
            self.starting_points[jj] =np.median(starting_points)

        for ii in range(0, self.num_variables):

            NLF_temp = np.zeros((1,self.number_demos*self.num_points))
            POS_temp = np.zeros((self.num_points,self.number_demos))

            for jj in range(0,len(demonstration_data)):

                demo = demonstration_data[jj]

                # Downsample to number of points in final function fitting
                pos_temp = demo[:,ii]
                interpolated = interp1d(np.arange(len(pos_temp)), pos_temp, axis=0, kind='cubic', fill_value='extrapolate')
                pos_interp = interpolated(np.linspace(0, len(pos_temp), self.num_points))

                # Finit differencing to get the desired velocity and acceleration to compute the force
                vel_interp = np.gradient(pos_interp)/self.dt
                acc_interp = np.gradient(vel_interp)/self.dt

                # Calculate the forcing required to produce this motion
                # Using the ODE and solving for f: F = x_ddot-k(x_final-x)+bx_dot
                NLF_interp = acc_interp-self.k*np.array([self.attractor_points[ii]-p for p in pos_interp])+self.b*vel_interp

                # The part of the forcing function to be learned is before it has been normalized
                # by the exponential (i.e., canonical system - s)
                Decay_Normalized_NLF = np.divide(NLF_interp,self.s)
                NLF_temp[0,jj*self.num_points:(jj+1)*self.num_points]=Decay_Normalized_NLF

                POS_temp[:,jj]=pos_interp

            self.desired_forcing.append(NLF_temp)
            self.kinematic_data.append(POS_temp)


    def computeDMP(self):

        # For each dimension, calculate a DMP

        self.weighting_fxns = np.zeros((self.num_variables, self.gmm_states))

        for xx in range(0,self.num_variables):

            # Initialize the Gaussian Mixture Model
            self.tbGMM = GMM()
            self.tbGMM.initialize(states=self.gmm_states,decay_fxn=self.s)

            for ii in range(0,self.tbGMM.states):
                self.tbGMM.sigma[ii]= 2e-5; # Heuristic value

            # Compute Gaussian Activation
            H = np.zeros((self.tbGMM.states,self.num_points))

            for ii in range(0,self.tbGMM.states):
                H[ii,:]=self.tbGMM.gaussianPDF(decay_fxn =self.s,mu=self.tbGMM.mu[ii],sigma=self.tbGMM.sigma[ii])

            H_sum = np.sum(H,0)

            for ii in range(0,self.tbGMM.states):
                H[ii,:]=np.divide(H[ii,:],H_sum)

            H_repeating = np.zeros((self.tbGMM.states,self.num_points*self.number_demos))

            for ii in range(0,self.number_demos):
                H_repeating[:,ii*self.num_points:(ii+1)*self.num_points]=H

            X = np.ones((self.num_points*self.number_demos,1))
            Y = np.transpose(self.desired_forcing[xx])

            for ii in range(0, self.tbGMM.states):
                W = np.diag(H_repeating[ii,:])
                self.weighting_fxns[xx,ii] = np.linalg.solve(np.matmul(np.matmul(np.transpose(X),W),X),np.matmul(np.matmul(np.transpose(X),W),Y))

            self.DMP_forcing.append(np.matmul(self.weighting_fxns[xx,:],H))

            # if (xx==0):
            #     plt.plot(np.linspace(0,1,200),H)
            #     plt.show()


    def getTrajectory(self):

        return_trajectory = []

        for xx in range(0, self.num_variables):
            currF = self.DMP_forcing[xx]

            # Initial conditions for numerical integration
            dx=0
            x = self.starting_points[xx]

            trajectory = np.zeros((self.num_points,1))

            for t in range(0,self.num_points):
                ddx = self.k*(self.attractor_points[xx]-x)-self.b*dx+currF[t]*self.s[t]
                dx = dx + ddx* self.dt
                x = x + dx*self.dt
                trajectory[t,0] = x
            return_trajectory.append(trajectory)

            #if xx==2:
            #    plt.plot(trajectory)
            #    plt.show()

        return return_trajectory

    def getForces(self):

        return_forces = []

        for xx in range(0, self.num_variables):
            return_forces.append(np.multiply(self.DMP_forcing[xx],self.s))

        return self.starting_points, self.attractor_points, return_forces


if __name__ == "__main__":
    print "HELLO"




