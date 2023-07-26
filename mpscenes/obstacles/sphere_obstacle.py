from dataclasses import dataclass
from typing import List, Optional
import os
import csv
import numpy as np
from omegaconf import OmegaConf

from mpscenes.obstacles.collision_obstacle import CollisionObstacle, CollisionObstacleConfig, GeometryConfig


@dataclass
class SphereGeometryConfig(GeometryConfig):
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and radius of a sphere obstacle.

    Parameters:
    ------------

    radius: float: Radius of the obstacle
    """

    radius: float = 1.0


@dataclass
class SphereObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a spherical obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """

    geometry: SphereGeometryConfig
    low: Optional[SphereGeometryConfig] = None
    high: Optional[SphereGeometryConfig] = None


class SphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        if 'schema' not in kwargs:
            schema = OmegaConf.structured(SphereObstacleConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        self.check_completeness()

    def size(self):
        return [
            self.radius(),
        ]

    def limit_low(self):
        if self._config.low:
            return [
                np.array(self._config.low.position),
                self._config.low.radius,
            ]
        else:
            return [np.ones(self.dimension()) * -1, 0]

    def limit_high(self):
        if self._config.high:
            return [
                np.array(self._config.high.position),
                self._config.high.radius,
            ]
        else:
            return [np.ones(self.dimension()) * 1, 1]

    def radius(self):
        return self._config.geometry.radius

    def shuffle(self):
        random_pos = np.random.uniform(
            self.limit_low()[0], self.limit_high()[0], self.dimension()
        )
        random_radius = np.random.uniform(
            self.limit_low()[1], self.limit_high()[1], 1
        )
        self._config.geometry.position = random_pos.tolist()
        self._config.geometry.radius = float(random_radius)


    def csv(self, file_name, samples=100):
        theta = np.arange(-np.pi, np.pi + np.pi / samples, step=np.pi / samples)
        x = self.position()[0] + (self.radius() - 0.1) * np.cos(theta)
        y = self.position()[1] + (self.radius() - 0.1) * np.sin(theta)
        with open(file_name, "w", encoding="utf8") as file:
            csv_writer = csv.writer(file, delimiter=",")
            for i in range(2 * samples):
                csv_writer.writerow([x[i], y[i]])

    def distance(self, position: np.ndarray) -> float:
        pos = self.position_into_obstacle_frame(position)
        return np.linalg.norm(pos, axis=0) - self.radius()
