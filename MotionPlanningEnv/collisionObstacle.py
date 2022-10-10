from abc import abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent

from dataclasses import dataclass

@dataclass
class CollisionObstacleConfig:
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the dimension and the type
    of collision obstacle.

    Parameters:
    ------------

    dimension : int : Dimension of the obstacle
    type : str : Type of the obstacle
    """
    type: str


class CollisionObstacle(MotionPlanningComponent):

    @abstractmethod
    def dimension(self):
        pass

    def type(self) -> str:
        return self._config.type

    def geometry(self):
        return self._config.geometry

    def evaluate(self, **kwargs) -> list:
        return [
            self.position(**kwargs),
            self.velocity(**kwargs),
            self.acceleration(**kwargs)
        ]

    @abstractmethod
    def position(self, **kwargs):
        pass

    @abstractmethod
    def velocity(self, **kwargs):
        pass

    @abstractmethod
    def acceleration(self, **kwargs):
        pass

    def update_bullet_position(self, pybullet, **kwargs):
        pass
