from abc import ABC
import yaml
from omegaconf import OmegaConf


class ComponentIncompleteError(Exception):
    pass


class DimensionNotSuitableForEnv(Exception):
    pass


class MotionPlanningComponent(ABC):

    def __init__(self, schema, **kwargs):
        if "content_dict" in kwargs and "name" in kwargs:
            self._content_dict = kwargs.get("content_dict")
            self._name = kwargs.get("name")
        elif "file_name" in kwargs:
            with open(kwargs.get("file_name"), "r") as stream:
                self._content_dict = yaml.safe_load(stream)
            self._name = self._content_dict["name"]
            del self._content_dict["name"]
        self._config = OmegaConf.create(self._content_dict)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self._required_keys = []

    def check_completeness(self):
        """
        Check if all mandatory keys are provided.
        """
        incomplete = False
        missing_keys = ""
        for key in self._required_keys:
            if key not in self._required_keys:
                incomplete = True
                missing_keys += key + ", "
        if incomplete:
            raise ComponentIncompleteError(
                    "Missing keys: {missing_keys[:-2]}")

    def add_required_keys(self, keys):
        self._required_keys.append(keys)

    def name(self):
        return self._name

    def dict(self):
        return OmegaConf.to_container(self._config)
