#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 13:04:33 2019

@author: jpiramirez
"""
import numpy as np
import matplotlib.pyplot as plt

def traj(ss, sf, ts, tf, pts):
    S = np.array([[ts**5, ts**4, ts**3, ts**2, ts, 1],
                  [tf**5, tf**4, tf**3, tf**2, tf, 1],
                  [5*ts**4, 4*ts**3, 3*ts**2, 2*ts, 1, 0],
                  [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0],
                  [20*ts**3, 12*ts**2, 6*ts, 2, 0, 0],
                  [20*tf**3, 12*tf**2, 6*tf, 2, 0, 0]])
    b = np.array([[ss],[sf],[0],[0],[0],[0]])
    x = np.linalg.solve(S, b)
    t = np.linspace(ts, tf, pts)
    s = np.polyval(x, t)
    xd = np.diag(np.diag([5, 4, 3, 2, 1])*x[0:5])
    xdd = np.diag(np.diag([20, 12, 6, 2])*x[0:4])
    sd = np.polyval(xd, t)
    sdd = np.polyval(xdd, t)
    return s, sd, sdd, t

s, sd, sdd, t = traj(1, 0, 0, 10, 100)
ss = np.copy(s)
tt = np.copy(t)
plt.figure(1)
plt.plot(t, s, t, sd, t, sdd)
s, sd, sdd, t = traj(0, -1, 10, 20, 100)
print(ss.shape)
print(s.shape)
ss = np.concatenate((ss,s))
tt = np.concatenate((tt,t))

plt.figure(2)
plt.plot(tt, ss)
plt.show()
    