from dataclasses import dataclass
from typing import List, Optional
import numpy as np
from omegaconf import OmegaConf

from mpscenes.obstacles.collision_obstacle import CollisionObstacle, CollisionObstacleConfig, GeometryConfig
from mpscenes.common.errors import ComponentIncompleteError, DimensionNotSuitableForEnv


@dataclass
class UrdfGeometryConfig(GeometryConfig):
    pass


@dataclass
class UrdfObstacleConfig:
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
    type: str
    geometry: UrdfGeometryConfig
    scaling: float = 1
    movable: bool = False
    low: Optional[UrdfGeometryConfig] = None
    high: Optional[UrdfGeometryConfig] = None


class UrdfObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(UrdfObstacleConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
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
                "Pybullet only supports three dimensional obstacles"
            )
        self._bullet_id = pybullet.loadURDF(
            fileName=self.urdf(),
            basePosition=self.position(),
            globalScaling=self._config.scaling
            )
        return self._bullet_id
