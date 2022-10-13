import os

from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    ComponentIncompleteError,
    DimensionNotSuitableForEnv,
)
import numpy as np

from dataclasses import dataclass
from omegaconf import OmegaConf
from typing import List, Optional


@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry for urdf obstacles.

    This configuration class holds information about position.

    Parameters:
    ------------

    position: list: Position of the obstacle
    """

    position: List[float]


@dataclass
class UrdfObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a spherical obstacle.

    Parameters:
    ------------

    urdf : str : Filename of the urdf
    geometry : GeometryConfig : Geometry of the obstacle
    low: GeometryConfig : Lower limit for randomization
    high: GeometryConfig : Upper limit for randomization

    """

    urdf: str
    geometry: GeometryConfig
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None


class UrdfObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(UrdfObstacleConfig)
        super().__init__(schema, **kwargs)
        self.check_completeness()

    def urdf(self):
        return self._config.urdf

    def position(self):
        return self._config.geometry.position

    def velocity(self):
        return np.zeros(3)

    def acceleration(self):
        return np.zeros(3)

    def dimension(self):
        return len(self._config.geometry.position)

    def add_to_bullet(self, pybullet) -> int:
        if self.dimension() != 3:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports two dimensional obstacles"
            )
        return pybullet.loadURDF(fileName=self.urdf(), basePosition=self.position())
