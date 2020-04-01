from .board_config import BaseConfig


class MarkerConfig(BaseConfig):
    def __init__(self, **kwargs):
        super(MarkerConfig, self).__init__(**kwargs)

    def _set_variables(self, **kwargs):
        super(MarkerConfig, self)._set_variables(**kwargs)
        self._type_id = 'M'
