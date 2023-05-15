from typing import List, Optional
from abc import abstractmethod
import numpy as np
from mpscenes.common.component import MPComponent

from dataclasses import dataclass, field

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position and orientation
    of an obstacle. This class is further specified for the other obstacles.

    Parameters:
    ------------

    position: list: Position of the obstacle
    orientation: list: Orientation of the obstacle
    """

    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])

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

    def orientation(self, **kwarg) -> np.ndarray:
        return np.array(self.geometry().orientation)

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self.geometry().position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def movable(self):
        return self._config.movable

    def position_into_obstacle_frame(self, position: np.ndarray) -> np.ndarray:
        return position - self.position()

    @abstractmethod
    def distance(self, position: np.array) -> float:
        pass

    @abstractmethod
    def size(self):
        pass
