from abc import ABC, abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent

class CollisionObstacle(MotionPlanningComponent):
    def __init__(self, name, obstDict):
        self._required_keys = [
            'dim',
            'type',
            'geometry',
        ]
        super().__init__(name, obstDict)

    def dim(self):
        return self._contentDict['dim']

    def type(self):
        return self._contentDict['type']

    def geometry(self):
        return self._contentDict['geometry']

    @abstractmethod
    def position(self, **kwargs):
        pass
