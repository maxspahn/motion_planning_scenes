from abc import ABC
import yaml
from omegaconf import OmegaConf

from mpscenes.common.errors import ComponentIncompleteError, DimensionNotSuitableForEnv


class MPComponent(ABC):

    def __init__(self, **kwargs):
        schema = kwargs.get('schema')
        if 'content_dict' in kwargs and 'name' in kwargs:
            self._content_dict = kwargs.get('content_dict')
            self._name = kwargs.get('name')
        elif 'file_name' in kwargs:
            with open(kwargs.get('file_name'), 'r') as stream:
                self._content_dict = yaml.safe_load(stream)
            self._name = self._content_dict['name']
            del self._content_dict['name']
        self._config = OmegaConf.create(self._content_dict)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)

    def check_completeness(self):
        pass
        """
        incomplete = False
        missingKeys = ""
        for key in self._required_keys:
            if key not in self._content_dict.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise ComponentIncompleteError("Missing keys: %s" % missingKeys[:-2])
        """

    def name(self):
        return self._name

    def evaluate_components(self, mask: list, t: float):
        result = {}
        for mask_entry in mask:
            try:
                result[mask_entry] = getattr(self, mask_entry)(t=t)
            except TypeError as _:
                result[mask_entry] = getattr(self, mask_entry)()
        return result

    def dict(self):
        return OmegaConf.to_container(self._config)

    def add_to_bullet(self, pybullet, position=[0.0, 0.0, 0.0]) -> int:
        """Adds component to pybullet instance.

        Returns
        ----------
        int: multi_body_identifier
        """
        pass
