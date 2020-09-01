""" Creates and updates a Time-based GMM

 Inspired by Calinon et al.
 See pbdlib
 Created: 02/20/2020
"""

__author__ = "Mike Hagenow"

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import mayavi.mlab as mlab
import os
from scipy.interpolate import interp1d

class GMM:

    # Number of Gaussians that comprise the mixture model
    states = 5;
    sigma_reg = 1e-4; # to avoid numerical instability

    # Calculated Variables
    mu = np.zeros((5,1));
    sigma = np.zeros((5,1))
    priors=np.zeros((5,1))

    def initialize(self, states=5, decay_fxn = None):
        self.states = states
        self.mu = np.zeros((states, 1));
        self.sigma = np.zeros((states, 1))
        self.priors = np.zeros((states, 1))

        # Space out the segments with respect to the decay function
        time_split = np.linspace(np.min(decay_fxn),np.max(decay_fxn),self.states+1)

        for ii in range(0,self.states):
            # Get just the data in the time slice for a particular gaussian
            data_temp = decay_fxn[(decay_fxn>=time_split[ii]) & (decay_fxn<time_split[ii+1])]
            self.priors[ii]=len(data_temp)
            self.mu[ii]=np.mean(data_temp)
            self.sigma[ii]=np.cov(data_temp)+0.0+self.sigma_reg

        sum_priors = np.sum(self.priors)
        self.priors = np.divide(self.priors,sum_priors)


    # Creates a gaussian function for a given function and gaussian parameters
    def gaussianPDF(self,decay_fxn = None,mu=0,sigma=0):
        fxn_diff = np.array([p-mu for p in decay_fxn])
        prob = np.multiply(np.divide(fxn_diff,sigma),fxn_diff.reshape((len(fxn_diff),1)))
        prob = np.array([np.exp(-0.5*probtemp) / np.sqrt((2*np.pi)*np.abs(sigma)+np.finfo(float).tiny) for probtemp in prob])
        return prob.reshape((len(fxn_diff),))



if __name__ == "__main__":
    print "HELLO"




