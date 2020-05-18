import cv2
from aruco_analyzer.marker_generator import generate_single_marker


for id in range(4):
    image = generate_single_marker(cv2.aruco.DICT_4X4_50, ar_id=id, length=490, margin=10)

    cv2.imshow('image', image)
    cv2.waitKey(0)

    cv2.imwrite('M{}.png'.format(id), image)
