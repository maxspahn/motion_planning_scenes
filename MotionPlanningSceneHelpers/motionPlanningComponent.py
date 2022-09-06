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
        # check nested mandatory keys
        for key in self._required_keys:
            if "." in key:
                nested_key = key.split(".")
                key_pointer = self._content_dict
                for nkey in nested_key:
                    if nkey in key_pointer:
                        key_pointer = key_pointer[nkey]
                    else:
                        incomplete = True
                        missing_keys += key + ", "
                        break
            # check mandatory keys
            else:
                if key not in self._content_dict.keys():
                    incomplete = True
                    missing_keys += key + ", "
        if incomplete:
            raise ComponentIncompleteError(
                    f"Missing keys: {missing_keys[:-2]}")

    def add_required_keys(self, keys, parent_keys=""):
        """
        Adds keys to list of required keys.

        allowed types are a list containing strings or dictionaries
        nested dictionaries are allowed as long as the most
        nested dictionary contains a list with strings.

        keys character "." is not allowed in keys
        """
        if isinstance(keys, dict):
            for (k, v) in keys.items():
                self.add_required_keys(v, parent_keys=parent_keys+k+".")

        elif isinstance(keys, list):
            for key in keys:
                if isinstance(key, dict):
                    for (k, v) in key.items():
                        self.add_required_keys(v, parent_keys=parent_keys+k+".")
                elif isinstance(key, str):
                    if "." in key:
                        raise ValueError("'.' not allowed in the key")
                    self._required_keys.append(parent_keys+key)
                else:
                    raise TypeError("only strings of "\
                    "dictionaries allowed in list")

    def name(self):
        return self._name

    def dict(self):
        return OmegaConf.to_container(self._config)
