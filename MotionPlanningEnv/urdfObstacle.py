import os

from MotionPlanningEnv.collisionObstacle import CollisionObstacle, CollisionObstacleConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv
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
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        schema = OmegaConf.structured(UrdfObstacleConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        self.checkGeometryCompleteness()

    def checkUrdfFile(self):
        if 'urdf' not in self._content_dict:
            raise ComponentIncompleteError("Missing urdf file")

    def urdf(self):
        return self._config.urdf

    def checkGeometryCompleteness(self):
        if 'position' not in self.geometry():
            raise ComponentIncompleteError("Missing position in geometry for urdf obstacle")

    def position(self):
        return self._config.geometry.position

    def velocity(self):
        return np.zeros(3)

    def acceleration(self):
        return np.zeros(3)

    def toDict(self):
        return OmegaConf.to_container(self._config)

    def add2Bullet(self, pybullet):
        if self._config.dim != 3:
            raise DimensionNotSuitableForEnv("Pybullet only supports two dimensional obstacles")
        pybullet.loadURDF(fileName=self.urdf(), basePosition=self.position())
