import cv2
import numpy as np

length = 10

number_of_cubes = 2
dictionaryName = cv2.aruco.DICT_4X4_50
dictionary = cv2.aruco.getPredefinedDictionary(dictionaryName)
markersX = 3
markersY = markersX
markerSeparation = 30
markerLength = 470
margins = markerSeparation
borderBits = 1

width = markersX*markerLength + (markersX+1)*markerSeparation
height = width

cube_ids = [markersX**2*4*i for i in range(number_of_cubes)]

for cube_id in cube_ids:
        print('Cube ID C{:03d}'.format(cube_id))
        cube_mat = np.zeros((2*height+3, 2*width+3))
        mats = list()
        
        for i in range(0, 1):
            board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary, cube_id+(markersX**2)*i)
            mat = board.draw((width, height), np.array([]), margins, borderBits)
            mats.append(mat)

        cube_mat[1:height+1, 1:width+1] = mats[0]
        cube_mat[1:height+1, width+2:-1] = mats[1]
        cube_mat[height+2:-1, 1:width+1] = mats[2]
        cube_mat[height+2:-1, width+2:-1] = mats[3]

        # cv2.imshow('cube', cube_mat)
        # cv2.waitKey(0)

        cv2.imwrite('{}X{}_C{:03d}.png'.format(length, length, cube_id), cube_mat)
