from .board_config import BoardConfig, transform_vector
from operator import add
import numpy as np
import cv2


class SatelliteBoardConfig(BoardConfig):

    def __init__(self, **kwargs):
        super(SatelliteBoardConfig, self).__init__(**kwargs)

        corners = []
        front_side_corners = []

        for j in range(self._marker_per_side, 0, -1):
            for i in range(1, self._marker_per_side+1):
                marker_corners = []
                marker_corners.append([0, (i-1) * self._marker_length + i*self._border, j*self._marker_length + j*self._border])
                marker_corners.append([0, i * self._marker_length + i*self._border, j*self._marker_length + j*self._border])
                marker_corners.append([0, i * self._marker_length + i*self._border, (j-1)*self._marker_length + j*self._border])
                marker_corners.append([0, (i-1) * self._marker_length + i*self._border, (j-1)*self._marker_length + j*self._border])
                front_side_corners.append(marker_corners)

        # transform coordinates so that centre of cube is (0,0,0)
        cube_length = self._marker_per_side*self._marker_length + (self._marker_per_side+1)*self._border
        transform = [cube_length/2, -cube_length/2, -cube_length/2]
        transformed_frontside_corners = []
        for marker_corners in front_side_corners:
            transformed_frontside_corners.append([list(map(add, transform, marker_corner)) for marker_corner in marker_corners])

        corners.extend(transformed_frontside_corners)

        # rotate coordinates for each cube side
        for marker_corners in transformed_frontside_corners:
            corners.append([transform_vector(marker_corner, (0, 0, 1), 90) for marker_corner in marker_corners])
        for marker_corners in transformed_frontside_corners:
            corners.append([transform_vector(marker_corner, (0, 1, 0), 90) for marker_corner in marker_corners])

        # transform into correct data format
        points = np.array([[np.array(marker_corners, 'float32')] for marker_corners in corners])
        ids = np.array([[id] for id in self._ids])

        self._board = cv2.aruco.Board_create(points, self._dictionary, ids)

    def _set_variables(self, **kwargs):
        super(SatelliteBoardConfig, self)._set_variables(**kwargs)
        self._type_id = 'S'
        self._border = kwargs['border']
        self._marker_per_side = kwargs['marker_per_side']
        self._first_marker = kwargs['first_marker']

        self._ids = [i for i in range(self._first_marker, self._first_marker+3*self._marker_per_side**2)]

    @property
    def first_marker(self):
        return self._first_marker

    @property
    def border(self):
        return self._border

    @property
    def ids(self):
        return self._ids
