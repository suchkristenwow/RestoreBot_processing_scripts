from matplotlib import pyplot as plt
import cv2 as cv
import numpy as np
import os 
from stitching import Stitcher,AffineStitcher


def plot_image(img, figsize_in_inches=(5,5)):
    fig, ax = plt.subplots(figsize=figsize_in_inches)
    ax.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
    ax.axis('off')
    plt.show()
    

img_dir = "/media/kristen/easystore3/matches/1650825680395175933/may_frames"
images = [os.path.join(img_dir, file) for file in os.listdir(img_dir) if os.path.isfile(os.path.join(img_dir, file))]
images = images[::2]

'''
stitcher = Stitcher(detector="sift", confidence_threshold=0.2)
panorama = stitcher.stitch(sampled_images)
'''

settings = {# The whole plan should be considered
            "crop": False,
            # The matches confidences aren't that good
            "confidence_threshold": 0.25}    

stitcher = AffineStitcher(**settings)
panorama = stitcher.stitch(images)

plot_image(panorama, (20,20))