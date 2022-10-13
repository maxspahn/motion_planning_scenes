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


class SphereObstacleMissmatchDimensionError(Exception):
    pass


@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and radius of a sphere obstacle.

    Parameters:
    ------------

    position: list: Position of the obstacle
    radius: float: Radius of the obstacle
    """

    position: List[float]
    radius: float


@dataclass
class SphereObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a spherical obstacle.

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


class SphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(SphereObstacleConfig)
        super().__init__(schema, **kwargs)
        self.check_completeness()

    def dimension(self):
        return len(self._config.geometry.position)

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

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self._config.geometry.position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

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
            pybullet.GEOM_SPHERE, radius=self.radius()
        )
        visual_shape_id = -1
        base_orientation = [0, 0, 0, 1]
        mass = int(self.movable())
        pybullet.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__))
        )
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="sphere_smooth.obj",
            rgbaColor=[1.0, 0.0, 0.0, 1.0],
            specularColor=[1.0, 0.5, 0.5],
            meshScale=[self.radius(), self.radius(), self.radius()],
        )
        assert isinstance(base_position, list)
        assert isinstance(base_orientation, list)
        return pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            base_position,
            base_orientation,
        )
