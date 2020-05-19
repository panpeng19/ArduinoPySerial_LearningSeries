# -*- coding: utf-8 -*-
"""
Created on Wed May 2020 12:45:44 2020

@author: Peng PAN
"""

import pickle
import serial			#importing pyserial package
import time			    #importing time 
import csv
import matplotlib.pyplot as plt		#importing matplotlib packages
import matplotlib.pyplot as plt, numpy as np, numpy.random, scipy
with open('mypicklefile13', 'rb') as f:
    # The protocol version used is detected automatically, so we do not
    # have to specify it.
    data = pickle.load(f).decode()
    
    print(data)
'''
data=[]
#t=time.localtime()
#histogram definition
xyrange = [[0,1000],[0,1000]] # data range
bins = [300,300] # number of bins
thresh = 1  #density threshold

#data definition
#N = 1e5
xdat, ydat  = data["X"], data["Y"]

# histogram the data
hh, locx, locy = scipy.histogram2d(xdat, ydat, range=xyrange, bins=bins)
posx = np.digitize(xdat, locx)
posy = np.digitize(ydat, locy)
plt.scatter(data,data)
					#plotting a graph
					#also you can plot a bargraph,boxplot by changing commands.
plt.title('distance vs time')
plt.xlabel("distance")
plt.ylabel("time")
plt.show()
'''