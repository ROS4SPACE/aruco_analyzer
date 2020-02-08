import cv2
import numpy as np

length = 10

number_of_grids = 1
dictionaryName = cv2.aruco.DICT_5X5_250
dictionary = cv2.aruco.getPredefinedDictionary(dictionaryName)
markersX = 4
markersY = markersX
markerSeparation = 20
markerLength = 470
margins = markerSeparation/2

borderBits = 1

width = 800
height = 800

grid_ids = [markersX*markersY*i for i in range(number_of_grids)]

for grid_id in grid_ids:
        print('Grid ID G{:03d}'.format(grid_id))

        board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary, grid_id)
        grid_mat = board.draw((width, height), np.array([]), margins, borderBits)

        # cv2.imshow('grid', grid_mat)
        # cv2.waitKey(0)
        
        cv2.imwrite('{}X{}_G{:03d}.png'.format(length, length, grid_id), grid_mat)
