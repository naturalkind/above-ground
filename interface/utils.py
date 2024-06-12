""" utils.py 

Contains a useful plotting function that is used in the coding examples.
The function was built using Plotly instead of Matplotlib due to its
interactive graphs and because it runs better on Raspberry Pi Linux.

Author: Eduardo Nigro
    rev 0.0.6
    2022-01-24
    
"""
import numpy as np
import plotly.io as pio
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import os
import sys
import cv2
import time

from matplotlib import pyplot as plt
from matplotlib import animation

class AnimTest(object):
    def __init__(self):
        self.fig = plt.figure(figsize=(10,10))
        self.ax = self.fig.add_subplot()
        self.ax.axis([0, 100, 0, 100])
        self.N = 100
        self.p_point = self.ax.scatter(0, 0, cmap='Greens')
        self.ani = animation.FuncAnimation(self.fig, self.update, self.N, fargs=(1, 1), interval=10000/self.N, blit=False)
        self.x = 2
        self.y = 3

    def update(self, num, a, b):
        self.x += 1
        self.y += 1
        self.p_point.set_offsets([self.x, self.y])
        self.p_point.set_array([self.x, self.y])
        print ([self.x, self.y])
        return self.p_point,

a = AnimTest()
plt.show()
