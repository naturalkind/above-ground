#import cv2
import PIL
import numpy as np
PIL.Image.MAX_IMAGE_PIXELS = 3586896448
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
img = mpimg.imread('media-info/orto.jpg')
imgplot = plt.imshow(img)
plt.show()

