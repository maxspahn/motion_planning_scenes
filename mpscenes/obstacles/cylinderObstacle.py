from dataclasses import dataclass
import numpy as np
import os
import csv
from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    DimensionNotSuitableForEnv,
)

from omegaconf import OmegaConf
from typing import List, Optional


class CylinderObstacleMissmatchDimensionError(Exception):
    pass


@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position,
    height and radius of a cylinder obstacle.

    Parameters:
    ------------

    position: list: Position of the obstacle
    radius: float: Radius of the obstacle
    height: float: Height of the obstacle
    """

    position: List[float]
    radius: float
    height: float


@dataclass
class CylinderObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for cylinder obstacle.

    This configuration class holds information about the position, size
    and randomization of a cylinder obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """

    geometry: GeometryConfig
    movable: bool = False
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None


class CylinderObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(CylinderObstacleConfig)
        super().__init__(schema, **kwargs)
        self.check_completeness()

    def dimension(self):
        return len(self._config.geometry.position)

    def limit_low(self):
        if self._config.low:
            return [
                np.array(self._config.low.position)
            ]
        else:
            return [np.ones(self.dimension()) * -1]

    def limit_high(self):
        if self._config.high:
            return [
                np.array(self._config.high.position)
            ]
        else:
            return [np.ones(self.dimension()) * 1]

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
        self._config.geometry.position = random_pos.tolist()

    def movable(self):
        return self._config.movable

    def csv(self, file_name, samples=100):
        theta = np.arange(-np.pi, np.pi + np.pi / samples, step=np.pi / samples)
        x = self.position()[0] + (self.radius() - 0.1) * np.cos(theta)
        y = self.position()[1] + (self.radius() - 0.1) * np.sin(theta)
        with open(file_name, mode="w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            for i in range(2 * samples):
                csv_writer.writerow([x[i], y[i]])

    def render_gym(self, viewer, rendering, **kwargs):
        if self.dimension() != 2:
            raise DimensionNotSuitableForEnv(
                "PlanarGym only supports two dimensional obstacles"
            )
        x = self.position()
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.radius())
        joint.add_attr(tf)

    def add_to_bullet(self, pybullet) -> int:
        if self.dimension() == 2:
            base_position = self.position().tolist() + [0.0]
        elif self.dimension() == 3:
            base_position = self.position().tolist()
        else:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports three dimensional obstacles"
            )
        collision_shape = pybullet.createCollisionShape(
            pybullet.GEOM_CYLINDER, radius=self.radius(), height=self.height()
        )
        visual_shape = pybullet.createVisualShape(
            pybullet.GEOM_CYLINDER, radius=self.radius(), length=self.height(), rgbaColor=[0.9,0,0.1,0.6]
        )
        base_orientation = [0, 0, 0, 1]
        mass = int(self.movable())
        assert isinstance(base_position, list)
        assert isinstance(base_orientation, list)
        return pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape,
            base_position,
            base_orientation,
        )
