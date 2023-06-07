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
 
 
#def match_features(key_points_for_all, descriptor_for_all, MRT, image_for_all, is_show=None):
#    """
#    match features
#    :param key_points_for_all:
#    :param descriptor_for_all:
#    :param MRT: Ratio Test
#    :param image_for_all:
#    :param is_show:
#    :return:
#    """
#    if is_show == None:
#        is_show = False
#    else:
#        is_show = True
# 
#    bf = cv2.BFMatcher(cv2.NORM_L2)
#    matches_for_all = []
#    for i in range(len(descriptor_for_all) - 1):
#        # Feature matching can try many ways
#        knn_matches = bf.knnMatch(descriptor_for_all[i], descriptor_for_all[i + 1], k=2)
#        good_matches = []
#        for m, n in knn_matches:
#            if m.distance < MRT * n.distance:
#                Good_matches.append ([m]) # Mainly here to add parentheses
# 
#        matches_for_all.append(np.array(good_matches))
# 
#        if is_show:
#            img_matches = np.empty((max(image_for_all[i].shape[0], image_for_all[i+1].shape[0]),
#                                    image_for_all[i].shape[1] + image_for_all[i+1].shape[1], 3), dtype=np.uint8)
#            cv2.drawMatchesKnn(image_for_all[i], key_points_for_all[i], image_for_all[i+1], key_points_for_all[i+1],
#                               good_matches, img_matches)
#            cv2.imshow("matches", img_matches)
#            cv2.waitKey(1500)
 

 
 
 
                          
                          
image_parts = DATA()
image_parts.parseIMG("cut_parts")
sift = cv2.xfeatures2d.SIFT_create()

#print (image_parts.file)
img_left = cv2.imread(image_parts.file["20_part"][0], 0)
features_left, left_descriptors = sift.detectAndCompute(img_left, None) # -> keypoints, descriptors

img_right = cv2.imread(image_parts.file["40_part"][0], 0)
features_right, right_descriptors = sift.detectAndCompute(img_right, None) # -> keypoints, descriptors
print (img_left.shape, len(features_left), len(features_right))

def show_futeres(img, features):
    # draw the detected key points
    sift_image = cv2.drawKeypoints(img, features, img)
    # show the image
    cv2.imshow('image', sift_image)
    # save the image
    cv2.waitKey(0)
    cv2.destroyAllWindows()

show_futeres(img_right, features_right)
show_futeres(img_left, features_left)
matcher = cv2.BFMatcher()


KNN = 2
LOWE = 0.7
TREES = 5
CHECKS = 50

matcher = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': TREES}, {'checks': CHECKS})
matches = matcher.knnMatch(left_descriptors, right_descriptors, k=KNN)

positive = []
for left_match, right_match in matches:
    if left_match.distance < LOWE * right_match.distance:
        positive.append([left_match])
#print (len(positive))#, matches[1][1].distance)

img_matches = np.empty((max(img_left.shape[0], img_right.shape[0]), 
                            img_left.shape[1] + img_right.shape[1], 3), dtype=np.uint8)
cv2.drawMatchesKnn(img_left, features_left, img_right, features_right, positive, img_matches) # None

#SIFT_matches = cv2.drawMatchesKnn(img_left, features_left, img_right, features_right, positive, None, flags=2)
#cv2.imshow("show", SIFT_matches)
cv2.imshow("show", img_matches)
cv2.waitKey(0)
cv2.destroyAllWindows()


