from .board_config import BoardConfig
import cv2


class GridBoardConfig(BoardConfig):

    def __init__(self, dictionary, marker_length, x, y, marker_separation, first_marker):
        super(GridBoardConfig, self).__init__(dictionary, marker_length)
        self._type = 'G'
        self._x = x
        self._y = y
        self._marker_separation = marker_separation
        self._first_marker = first_marker
        self._board = cv2.aruco.GridBoard_create(self._x, self._y, self._marker_length, self._marker_separation, self._dictionary, self._first_marker)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def first_marker(self):
        return self._first_marker

    @property
    def ids(self):
        return range(self._first_marker, self._first_marker + self._x*self._y)

    @property
    def id(self):
        return '{}{:03d}'.format(self.type, self._first_marker)
