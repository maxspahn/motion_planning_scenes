from abc import abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent

from dataclasses import dataclass

@dataclass
class CollisionObstacleConfig:
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the dimension and 
    the type of collision obstacle.

    Parameters:
    ------------

    dim : int : Dimension of the obstacle
    type : str : Type of the obstacle
    """
    dim: int
    type: str


class CollisionObstacle(MotionPlanningComponent):
    def __init__(self, **kwargs):
        self._required_keys = [
            'dim',
            'type',
            'geometry',
        ]
        super().__init__(**kwargs)

    def dim(self):
        return self._config.dim

    def type(self):
        return self._config.type

    def geometry(self):
        return self._config.geometry

    @abstractmethod
    def position(self, **kwargs):
        pass

    @abstractmethod
    def velocity(self, **kwargs):
        pass

    @abstractmethod
    def acceleration(self, **kwargs):
        pass

    def updateBulletPosition(self, pybullet, **kwargs):
        pass
