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

#print (image_parts.file)
img_left = cv2.imread(image_parts.file["20_part"][0])#, 0)
features_left, left_descriptors = sift.detectAndCompute(img_left, None) # -> keypoints, descriptors

img_right = cv2.imread(image_parts.file["40_part"][0])#, 0)
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

KNN = 2
LOWE = 0.7
TREES = 5
CHECKS = 50

matcher = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': TREES}, {'checks': CHECKS})
matches = matcher.knnMatch(left_descriptors, right_descriptors, k=KNN)

positive = []
right_matchs = []
left_matchs = []
good = []
for left_match, right_match in matches:
    if left_match.distance < LOWE * right_match.distance:
        print (dir(left_match), left_match.distance)
        left_matchs.append(left_match)
        right_matchs.append(right_match)
        positive.append([left_match])
        good.append(left_match)
#print (len(positive))#, matches[1][1].distance)


SIFT_matches = cv2.drawMatchesKnn(img_left, features_left, img_right, features_right, positive, None, flags=2)
cv2.imshow("show", SIFT_matches)
cv2.waitKey(0)
cv2.destroyAllWindows()

#Вычисляет оптимальное ограниченное аффинное преобразование с 4 степенями свободы между двумя наборами двумерных точек.
# (InputArray from, InputArray to, OutputArray inliers=noArray(), int method=RANSAC, double ransacReprojThreshold=3, size_t maxIters=2000, двойное доверие=0,99, size_t RefineIters=10)


left_matchs = np.float32([features_left[m.queryIdx].pt for m in good]).reshape(-1,1,2)
right_matchs = np.float32([features_right[m.trainIdx].pt for m in good]).reshape(-1,1,2)

#H, _ = cv2.estimateAffinePartial2D(right_matchs, left_matchs, False)
H, _ = cv2.findHomography(right_matchs, left_matchs, cv2.RANSAC, 5.0)
print (H, img_left.shape)


#h, w, c = img_right.shape
#warped = cv2.warpPerspective(img_left, H, (w, h), 
#                             borderMode=cv2.BORDER_CONSTANT, 
#                             borderValue=(0, 0, 0, 0))
                             

#warped = cv2.warpAffine(img_left, H, (w, h))

#warped = cv2.perspectiveTransform(temp_points, H)

warped = warpImages(img_left, img_right, H)
print (warped.shape)
#output = np.zeros((h, w, 3), np.uint8)
#alpha = warped[:, :, 2] / 255.0
#output[:, :, 0] = (1. - alpha) * img_right[:, :, 0] + alpha * warped[:, :, 0]
#output[:, :, 1] = (1. - alpha) * img_right[:, :, 1] + alpha * warped[:, :, 1]
#output[:, :, 2] = (1. - alpha) * img_right[:, :, 2] + alpha * warped[:, :, 2]

cv2.imshow("show", warped)
cv2.waitKey(0)
cv2.destroyAllWindows()

