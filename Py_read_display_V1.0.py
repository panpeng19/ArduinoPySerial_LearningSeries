import pickle        
import csv
import array as arr
import numpy as np
import pandas as pd

your_pickle_obj = pickle.loads(open('pickle_fake1', 'rb').read())

a=arr.array('h',your_pickle_obj)
data = np.array(a)
data = data.astype(np.uint32)
Q1=data[:-2:6]
Q2=data[1:-2:6]
Q3=data[2::6]
Q4=data[3::6]
Num_event=data[4::6]
#print(Q1,Q2,Q3,Q4,Num_event)

Sigma_Q=Q1+Q2+Q3+Q4
QL=Q1+Q3
QR=Q2+Q4
QU=Q1+Q2
QD=Q3+Q4
X=((QR-QL)<<10)/Sigma_Q
Y=((QU-QD)<<10)/Sigma_Q

X = X.astype(int)
Y = Y.astype(int)
#print(X,Y)
'''''''''''''''''''''''''''''''''''''''''''pandas'''''''''''''''''''''''''''''''''''''''''''



df = pd.DataFrame({
       "Q1":Q1,
       "Q2":Q2,         
       "Q3":Q3,      
       "Q4":Q4,
       "Num_event":Num_event,
       "Sigma_Q":Sigma_Q,
       "X":X,
       "Y":Y,
})

df.to_csv("output_array1.csv")

'''''''''''''''''''''''''''''''''''''''''''XY_Display'''''''''''''''''''''''''''''''''''''''''''
import matplotlib.pyplot as plt, numpy as np, numpy.random, scipy

#histogram definition
xyrange = [[0,1024],[0,1024]] # data range
bins = [100,100] # number of bins
thresh = 1  #density threshold

#data definition
#N = 1e5
#xdat, ydat = np.random.normal(size=N), np.random.normal(1.0, 0.6, size=N)
xdat, ydat  = df["X"], df["Y"]

# histogram the data
hh, locx, locy = scipy.histogram2d(xdat, ydat, range=xyrange, bins=bins)
posx = np.digitize(xdat, locx)
posy = np.digitize(ydat, locy)

#select points within the histogram
ind = (posx > 0) & (posx <= bins[0]) & (posy > 0) & (posy <= bins[1])
hhsub = hh[posx[ind] - 1, posy[ind] - 1] # values of the histogram where the points are
xdat1 = xdat[ind][hhsub < thresh] # low density points
ydat1 = ydat[ind][hhsub < thresh]
hh[hh < thresh] = np.nan # fill the areas with low density by NaNs

plt.imshow(np.flipud(hh.T),cmap='jet',extent=np.array(xyrange).flatten(), interpolation='none', origin='upper')
plt.colorbar()   
plt.plot(xdat1, ydat1, '.',color='darkblue')
plt.title('2D-PSD') # plot title will be shown above the plot. accepts a string as an argument.
plt.xlabel('x coordinate')
plt.ylabel('Y coordinate')
plt.show()
