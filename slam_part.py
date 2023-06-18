from typing import Union, List, Optional, Tuple
from tkinter import *
from tkinter import filedialog
from tkinter import messagebox
from tkinter import ttk
from PIL import Image, ImageTk
import os
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
import math
import time
from scipy import interpolate
from sklearn.cluster import SpectralClustering
import scipy.sparse.csr as csr
import scipy.sparse.csgraph as csgraph

class DATA(object):
    def __init__(self):
       self.file = []
    def parseIMG(self, dir_name):
        path = dir_name+"/"
        for r, d, f in os.walk(path):
            for ix, file in enumerate(f):
                self.file.append(os.path.join(r, file))
                
def find_matches(im1, im2, matcher):
    matches = matcher.knnMatch(im1.feat, im2.feat, k=2)
    good = [i for i, j in matches if i.distance < 0.7 * j.distance]
    return len(good), len(matches)

def find_keypoints(step,img):
    row = img.shape[0]
    col = img.shape[1]
    arr = []
    for i in range(0,col,step):
        for j in range(0,row,step):
            keypoint = cv2.KeyPoint(i,j,step)
            arr.append(keypoint)
    return arr

def find_descriptors(img,step,keypoints):
    ans = []
    intensityKeypoints = []
    for i in range(len(keypoints)):
        arr = np.zeros((step,step))
        x1,y1 = keypoints[i].pt
        row = img.shape[0]
        col = img.shape[1]
        if (x1+step <= row and y1+step <= col):
            intensityKeypoints.append(keypoints[i])
            x1,y1 = int(x1),int(y1)
            arr = np.float32(img[x1:x1 + step, y1:y1 + step].flatten())
            mean = np.mean(arr)
            arr = arr-mean
            div = np.sqrt(np.sum(np.multiply(arr,arr)))
            if (div == 1):
                arr = arr/pow(div,0.5)
            ans.append(arr/div)
    return ans,intensityKeypoints

class _StitchImage:
    _lastIdx = 1

    def __init__(self, image, name: str=None):
        self.image = image
        self.kp = None
        self.feat = None
        self.feature_finder = cv2.xfeatures2d.SIFT_create()

        if name is None:
            name = '%02d' % (_StitchImage._lastIdx)
            _StitchImage._lastIdx += 1
        self.name = name

    def find_features(self):
        print('Finding features for image', self.name)
        self.kp, self.feat = self.feature_finder.detectAndCompute(self.image, None)

    def find_dense_sift_features(self):
        print('Finding Dense sift features for image',self.name)
        self.kp = find_keypoints(15,self.image)
        self.feat = self.feature_finder.compute(self.image,self.kp)

    def find_window_based_correlation_features(self):
        print('Finding window based correlation features for image',self.name)
        keypoints = find_keypoints(5,self.image)
        self.feat, self.kp = find_descriptors(self.image, 5, keypoints)

def find_clusters(inputs, num_clusters):
    marking = {}
    for i,image in enumerate(inputs):
        if isinstance(image, str):
            img = cv2.cvtColor(cv2.imread(image), cv2.COLOR_BGR2RGBA)
        if img.shape[-1] == 3:
            img = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)

        fname = os.path.splitext(os.path.split(image)[1])[0]
        image_f = _StitchImage(img, name=fname)
        # find features
        image_f.find_features()
        marking[i] = image_f

    edge_matrix = []
    matcher = cv2.BFMatcher_create(cv2.NORM_L2)
    for i in range(len(marking)):
        temp = []
        for j in range(len(marking)):
            good, matches = find_matches(marking[i], marking[j], matcher)
            if matches == 0:
                temp.append(0)
            else:
                if good > 0.1 * matches:
                    temp.append(1)
                else:
                    temp.append(0)
        edge_matrix.append(temp)
    adjacency_matrix = np.array(edge_matrix)
    w, v = np.linalg.eig(adjacency_matrix)
    sc = SpectralClustering(num_clusters, affinity='precomputed', n_init=100)
    sc.fit(adjacency_matrix)
    groups = {}
    for i in range(len(sc.labels_)):
        try:
            groups[sc.labels_[i]].append(i)
        except:
            groups[sc.labels_[i]] = [i]
    clusters = []
    for g in groups:
        clusters.append([inputs[x] for x in groups[g]])
    print("@@@@@@@@@@@@@@@@@@@@@@", groups, clusters)
    return clusters

                          
image_parts = DATA()
image_parts.parseIMG("cut_parts_")
#random.shuffle(image_parts.file)
#print (image_parts.file)

clusters = find_clusters(image_parts.file, len(image_parts.file)//2)

print (clusters)
#for ix, file_name in enumerate(image_parts.file):
#    print (file_name)

