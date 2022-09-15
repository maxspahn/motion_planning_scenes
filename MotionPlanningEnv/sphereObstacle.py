from dataclasses import dataclass, KW_ONLY
import numpy as np
import os
import csv
from MotionPlanningEnv.freeCollisionObstacle import FreeCollisionObstacleConfig, FreeCollisionObstacle

from MotionPlanningSceneHelpers.motionPlanningComponent import (
    DimensionNotSuitableForEnv,
)

from omegaconf import OmegaConf
from typing import List, Optional

class SphereObstacleMissmatchDimensionError(Exception):
    pass

@dataclass
class GeometryConfig:
    """
    Configuration dataclass for geometry.

    This configuration class holds information and radius.

    Parameters:
    ------------

    radius: float: Radius of the obstacle
    """
    radius: float


@dataclass
class SphereObstacleConfig(FreeCollisionObstacleConfig):
    """
    Configuration dataclass for sphere obstacle.

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
    _: KW_ONLY
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None

class SphereObstacle(FreeCollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(SphereObstacleConfig)
        super().__init__(schema, **kwargs)

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
        self._config.position = random_pos.tolist()
        self._config.geometry.radius = float(random_radius)

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

    def add_to_bullet(self, pybullet):
        pybullet.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__))
        )

        mass = -1
        if self.movable():
            mass = self.mass()

        collision_shape = pybullet.createCollisionShape(
            pybullet.GEOM_SPHERE, radius=self.radius()
        )
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="sphere_smooth.obj",
            rgbaColor=self.color(),
            specularColor=[1.0, 0.5, 0.5],
            meshScale=[self.radius(), self.radius(), self.radius()],
        )
        if self.dimension() == 2:
            base_position = self.position() + [0.0]
        elif self.dimension() == 3:
            base_position = self.position()
        else:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports three dimensional obstacles"
            )
        base_orientation = self.orientation()

        pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            base_position,
            base_orientation,
        )
