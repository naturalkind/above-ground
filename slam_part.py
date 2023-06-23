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
from scipy.sparse import csr_matrix


def imgs(x):
    cv2.imshow('image', np.array(x))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


class DATA(object):
   def __init__(self):
       self.file = {}
   def parseIMG(self, dir_name):
       path = dir_name+"/"
       for r, d, f in os.walk(path):
           for ix, file in enumerate(f):
              self.file[int(file.split(".")[0].split('_')[0])] = os.path.join(r, file)

def warpImages(img1, img2, H):
    rows1, cols1 = img1.shape[:2]
    rows2, cols2 = img2.shape[:2]

    list_of_points_1 = np.float32([[0,0], [0, rows1],[cols1, rows1], [cols1, 0]]).reshape(-1, 1, 2)
    temp_points = np.float32([[0,0], [0,rows2], [cols2,rows2], [cols2,0]]).reshape(-1,1,2)

    # When we have established a homography we need to warp perspective
    # Change field of view
    list_of_points_2 = cv2.perspectiveTransform(temp_points, H)

    list_of_points = np.concatenate((list_of_points_1,list_of_points_2), axis=0)

    [x_min, y_min] = np.int32(list_of_points.min(axis=0).ravel() - 0.5)
    [x_max, y_max] = np.int32(list_of_points.max(axis=0).ravel() + 0.5)

    translation_dist = [-x_min,-y_min]

    H_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])

    output_img = cv2.warpPerspective(img2, H_translation.dot(H), 
                                    (x_max-x_min, y_max-y_min))
    output_img[translation_dist[1]:rows1+translation_dist[1],
               translation_dist[0]:cols1+translation_dist[0]] = img1

    return output_img       

def cameraPoseFromHomography(H):
    H1 = H[:, 0]
    H2 = H[:, 1]
    H3 = np.cross(H1, H2)

    norm1 = np.linalg.norm(H1)
    norm2 = np.linalg.norm(H2)
    tnorm = (norm1 + norm2) / 2.0;

    T = H[:, 2] / tnorm
    return np.mat([H1, H2, H3, T])            


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

                          
image_parts = DATA()
image_parts.parseIMG("data/cut_parts")
list_keys = sorted(list(image_parts.file.keys()))

sift = cv2.xfeatures2d.SIFT_create()

KNN = 2
LOWE = 0.7
TREES = 5
CHECKS = 50

matcher = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': TREES}, {'checks': CHECKS})

G = {}

for ix, i in enumerate(list_keys):
    if len(list_keys) > ix+1:
        # получить изображения
        img_path_left = image_parts.file[list_keys[ix]]
        img_path_right = image_parts.file[list_keys[ix+1]]

        img_left = cv2.imread(img_path_left)
        img_right = cv2.imread(img_path_right)
        
        # получить дискрипторы изображений
        features_left, left_descriptors = sift.detectAndCompute(img_left, None)
        features_right, right_descriptors = sift.detectAndCompute(img_right, None)
                
        # сравнить        
        matches = matcher.knnMatch(left_descriptors, right_descriptors, k=KNN)
        good = []
        for left_match, right_match in matches:
            if left_match.distance < LOWE * right_match.distance:
                good.append(left_match)

        if int(100*(len(good)/len(matches))) > 40:
            left_matchs = np.float32([features_left[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            right_matchs = np.float32([features_right[m.trainIdx].pt for m in good]).reshape(-1,1,2)
            H, _ = cv2.findHomography(right_matchs, left_matchs, cv2.RANSAC, 5.0)
#            warped = warpImages(img_left, img_right, H)
#            print (warped.shape)
#            imgs(warped)              
            G[(list_keys[ix], list_keys[ix+1])] = H
            
            
            # тестирование
#            cam_pos = cameraPoseFromHomography(H)

#            angle = math.atan2(H[1,0], H[0,0])



            #euler_rot = rotationMatrixToEulerAngles(H)
#            matches_mask = _.ravel().tolist()
#            print (cam_pos.shape, H.shape, len(matches_mask))
#            warped = warpImages(img_left, img_right, cam_pos[:,:])
#            print (warped.shape)
#            imgs(warped)     
            print (angle)                   
            

#list_keys = sorted(list(G.keys()))

#for ix, i in enumerate(list_keys):
#    if len(list_keys) > ix+1:
#        print (list_keys[ix], list_keys[ix+1])




#dst = cv2.convertPointsFromHomogeneous(H)
#Предполагая H в качестве матрицы гомографии и K в качестве матрицы камеры, код Python:

#num, Rs, Ts, Ns  = cv2.decomposeHomographyMat(H, K)
#число возможных решений будет возвращено.

#Rs содержит список матрицы вращения.
#Ts содержит список векторов перевода.
#Ns содержит список векторов нормалей к плоскости.

#Для получения дополнительной информации ознакомьтесь с официальной документацией:
#OpenCV 3.4 - decomposeHomographyMat()





