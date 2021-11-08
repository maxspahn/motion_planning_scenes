from abc import ABC, abstractmethod

class ComponentIncompleteError(Exception):
    pass

class MotionPlanningComponent(ABC):

    def __init__(self, name, contentDict):
        self._contentDict = contentDict
        self._name = name

    def checkCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._required_keys:
            if key not in self._contentDict.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise ComponentIncompleteError("Missing keys: %s" % missingKeys[:-2])

    def name(self):
        return self._name

    @abstractmethod
    def toDict(self):
        pass
