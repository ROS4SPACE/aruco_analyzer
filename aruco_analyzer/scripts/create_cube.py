import cv2
from aruco_analyzer.marker_generator import generate_cube


number_of_cubes = 1
dictionary = cv2.aruco.DICT_4X4_250
x = 2
separation = 30
length = 470
margin = 0
borderBits = 1
cube_ids = [x**2*4*i for i in range(number_of_cubes)]

for cube_id in cube_ids:
    images = generate_cube(dictionary, first_ar_id=0, x=x, length=length, separation=separation, margin=margin)

    for side, image in images.items():
        cv2.imshow('grid', image)
        cv2.waitKey(0)
        cv2.imwrite('{}X{}_G{:03d}_{}.png'.format(x, x, cube_id, side), image)
