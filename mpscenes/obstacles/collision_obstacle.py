from typing import List, Optional
from abc import abstractmethod
import numpy as np
from mpscenes.common.component import MPComponent

from dataclasses import dataclass

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    of an obstacle. This class is further specified for the other obstacles.

    Parameters:
    ------------

    position: list: Position of the obstacle
    """

    position: List[float]

@dataclass
class CollisionObstacleConfig:
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the dimension and the type
    of collision obstacle.

    Parameters:
    ------------

    type : str : Type of the obstacle
    geometry : GeometryConfig : Geometry of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """

    type: str
    geometry: GeometryConfig
    movable: bool = False
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None


class CollisionObstacle(MPComponent):

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

    def dimension(self):
        return len(self.geometry().position)

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self.geometry().position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def movable(self):
        return self._config.movable

    def update_bullet_position(self, pybullet, **kwargs):
        pass
