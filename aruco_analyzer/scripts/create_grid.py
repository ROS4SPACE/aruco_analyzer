import cv2
from aruco_analyzer.marker_generator import generate_grid


number_of_grids = 1
dictionary = cv2.aruco.DICT_4X4_250
x = 2
y = 2
separation = 30
length = 470
margin = 0
borderBits = 1
grid_ids = [x*y*i for i in range(number_of_grids)]

for grid_id in grid_ids:
    image = generate_grid(dictionary, first_ar_id=0, x=x, y=y, length=length, separation=separation, margin=margin)

    cv2.imshow('grid', image)
    cv2.waitKey(0)

    cv2.imwrite('{}X{}_G{:03d}.png'.format(x, y, grid_id), image)
