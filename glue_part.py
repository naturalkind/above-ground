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

class DATA(object):
   def __init__(self):
       self.file = {}
   def parseIMG(self, dir_name):
       path = dir_name+"/"
       for r, d, f in os.walk(path):
           for ix, file in enumerate(f):
                      if ".png" in file.lower():
                          self.file[file.split(".")[0]] = [os.path.join(r, file)]
                      if ".jpg" in file.lower(): 
                          self.file[file.split(".")[0]] = [os.path.join(r, file)]
                      if ".jpeg" in file.lower(): 
                          self.file[file.split(".")[0]] = [os.path.join(r, file)]
 
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
                          
image_parts = DATA()
image_parts.parseIMG("cut_parts")
sift = cv2.xfeatures2d.SIFT_create()

KNN = 2
LOWE = 0.7
TREES = 5
CHECKS = 50

matcher = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': TREES}, {'checks': CHECKS})


"""
обработать все изображения
получить дискрипторы
сравнить
создавать
"""
keys_list_img = list(image_parts.file.keys())
random.shuffle(keys_list_img)
G = {}
print (len(keys_list_img))
def func_rec(ID):
    for ix, i in enumerate(keys_list_img):
#        del image_parts.file[i]
#        del keys_list_img[ix]
        
        img_dir_left = image_parts.file[keys_list_img[ix]][0]
        img_left = cv2.imread(img_dir_left)
        features_left, left_descriptors = sift.detectAndCompute(img_left, None)
        for iix, ii in enumerate(keys_list_img[ix:]):
                img_dir_right = image_parts.file[keys_list_img[iix]][0]
                img_right = cv2.imread(img_dir_right)
                features_right, right_descriptors = sift.detectAndCompute(img_right, None)
                
                matches = matcher.knnMatch(left_descriptors, right_descriptors, k=KNN)
                positive = []
                good = []
                for left_match, right_match in matches:
                    if left_match.distance < LOWE * right_match.distance:
                        positive.append([left_match])
                        good.append(left_match)

                if len(good) > 40:
                    if keys_list_img[ix] != keys_list_img[iix]:
                        print (keys_list_img[ix], keys_list_img[iix])
    #                    del image_parts.file[ii]
    #                    del keys_list_img[iix]
                        
                        left_matchs = np.float32([features_left[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                        right_matchs = np.float32([features_right[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                        H, _ = cv2.findHomography(right_matchs, left_matchs, cv2.RANSAC, 5.0)

                        warped = warpImages(img_left, img_right, H)
                        print (warped.shape)
                        G[ID] = warped
                        cv2.imshow("show", warped)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()                    
        ID += 1

func_rec(0)
#print (G)



