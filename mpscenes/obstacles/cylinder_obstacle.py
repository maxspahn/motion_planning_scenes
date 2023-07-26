from dataclasses import dataclass
from typing import Optional
import os
import numpy as np
from omegaconf import OmegaConf

from mpscenes.obstacles.collision_obstacle import CollisionObstacle, CollisionObstacleConfig, GeometryConfig
from mpscenes.common.errors import DimensionNotSuitableForEnv


@dataclass
class CylinderGeometryConfig(GeometryConfig):
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and size of the cylinder obstacle.

    Parameters:
    ------------

    radius: float: Radius of the cylinder
    height: float: Height of the cylinder
    """

    radius: float = 1.0
    height: float = 1.0


@dataclass
class CylinderObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for box obstacle.

    This configuration class holds information about the position, size
    and randomization of a cylinder obstacle.

    Parameters:
    ------------

    geometry : CylinderGeometryConfig : Geometry of the box
    low : CylinderGeometryConfig : Lower limit for randomization
    high : CylinderGeometryConfig : Upper limit for randomization
    """

    geometry: CylinderGeometryConfig
    low: Optional[CylinderGeometryConfig] = None
    high: Optional[CylinderGeometryConfig] = None


class CylinderObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(CylinderObstacleConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        self.check_completeness()

    def size(self):
        return [
            self.radius(),
            self.height(),
        ]

    def dimension(self):
        return len(self._config.geometry.position)

    def limit_low(self):
        if self._config.low:
            return [
                np.array(self._config.low.position),
                self._config.low.radius,
                self._config.low.height,
            ]
        else:
            return [np.ones(self.dimension()) * -1, 0, 0]

    def limit_high(self):
        if self._config.high:
            return [
                np.array(self._config.high.position),
                self._config.high.radius,
                self._config.high.height,
            ]
        else:
            return [np.ones(self.dimension()) * 1, 1, 1]

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self._config.geometry.position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def radius(self):
        return self._config.geometry.radius

    def height(self):
        return self._config.geometry.height

    def shuffle(self):
        random_pos = np.random.uniform(
            self.limit_low()[0], self.limit_high()[0], self.dimension()
        )
        random_radius = np.random.uniform(
            self.limit_low()[1], self.limit_high()[1], 1
        )
        random_height = np.random.uniform(
            self.limit_low()[2], self.limit_high()[2], 1
        )
        self._config.geometry.position = random_pos.tolist()
        self._config.geometry.radius= float(random_radius)
        self._config.geometry.height = float(random_height)

    def movable(self):
        return self._config.movable

    def csv(self, file_name, samples=100):
        pass

    def distance(self, position) -> float:
        pos = self.position_into_obstacle_frame(position)
        if len(pos.shape) > 1:
            pos[2, :] += self.size()[1]/2
            norm_pos_2 = np.linalg.norm(pos[0:2, :], axis=0)
            abs_val = np.absolute(np.stack((norm_pos_2, pos[2, :])))
            d = np.transpose(np.subtract(np.transpose(abs_val), np.array(self.size())))
            return np.minimum(np.maximum(d[0, :], d[1, :]), 0.0) + np.linalg.norm(np.maximum(d, 0.0), axis=0)
        else:
            pos[2] += self.size()[1]/2
            d = np.absolute(np.array([np.linalg.norm(pos[0:2]), pos[2]])) - np.array(self.size())
            return np.minimum(np.maximum(d[0], d[1]), 0.0) + np.linalg.norm(np.maximum(d, 0.0))
