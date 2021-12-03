from abc import abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent


class CollisionObstacle(MotionPlanningComponent):
    def __init__(self, **kwargs):
        self._required_keys = [
            'dim',
            'type',
            'geometry',
        ]
        super().__init__(**kwargs)

    def dim(self):
        return self._contentDict['dim']

    def type(self):
        return self._contentDict['type']

    def geometry(self):
        return self._contentDict['geometry']

    @abstractmethod
    def position(self, **kwargs):
        pass

    def updateBulletPosition(self, pybullet, **kwargs):
        pass
