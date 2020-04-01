from .board_config import BoardConfig
import cv2


class GridBoardConfig(BoardConfig):

    def __init__(self, **kwargs):
        super(GridBoardConfig, self).__init__(**kwargs)

        self._board = cv2.aruco.GridBoard_create(self._x, self._y, self._marker_length, self._marker_separation, self._dictionary, self._first_marker)

    def _set_variables(self, **kwargs):
        super(GridBoardConfig, self)._set_variables(**kwargs)
        self._type_id = 'G'
        self._x = kwargs['x']
        self._y = kwargs['y']
        self._marker_separation = kwargs['marker_separation']
        self._first_marker = kwargs['first_marker']

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
