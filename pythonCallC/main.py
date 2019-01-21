# -*- coding: utf-8 -*-
import ctypes
import numpy as np
import random 
import matplotlib
#lib_0 = ctypes.cdll.LoadLibrary("/usr/local/lib/libtest_filter.so")
lib = ctypes.cdll.LoadLibrary("/usr/local/lib/libtest_filter.so")

time = np.zeros(200)
step = 1.0
y = np.zeros(time.shape)
k = 1.5
b = 0.2
sigma = 0


# generate test data
for i in range(*time.shape):
    time[i] = i*step 
    y[i] = k*time[i]+b+random.uniform(-sigma,sigma)
    
#matplotlib.pyplot.plot(time,y,'.')
    

# run ransac in c
n_samples = 20
n_iterations = 100
error_threshold = 1.0
targets_x = (ctypes.c_float * len(y))(*y)
Dimension = 1
samples = (ctypes.c_float * len(time))(*time)
count = len(time)
temp = [0.0,0.0]
params_x = (ctypes.c_float * len(temp))(*temp)
data1 = (ctypes.c_float * 1)()
c_float_p = ctypes.POINTER(ctypes.c_float)
lib.RANSAC_linear_model( ctypes.c_int(n_samples), ctypes.c_int(n_iterations),  \
ctypes.c_float(error_threshold),targets_x, ctypes.c_int(Dimension), samples, ctypes.c_int(count), params_x, ctypes.cast(data1, c_float_p));




temp =1