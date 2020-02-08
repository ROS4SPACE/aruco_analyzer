import os
import numpy as np
import cv2

for id in range(4):
    pixels = 200
    border = 10
    dictionaryName = cv2.aruco.DICT_5X5_50

    dictionary = cv2.aruco.getPredefinedDictionary(dictionaryName)

    marker = cv2.aruco.drawMarker(dictionary, id, pixels)

    image = np.full((pixels+2*border, pixels+2*border), fill_value=255, dtype=np.uint8)
    image[border:border+marker.shape[0], border:border+marker.shape[1]] = marker

    # cv2.imshow('image', image)
    # cv2.waitKey(0)
    
    cv2.imwrite('M{}.png'.format(id), image)
    