from dataclasses import dataclass, field
import csv
from typing import List, Optional
import numpy as np

import os
from omegaconf import OmegaConf

from MotionPlanningSceneHelpers.motionPlanningComponent import (
    DimensionNotSuitableForEnv,
)
from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)

@dataclass
class GeometryConfig:
    """
    This configuration class holds information about
    the geometry of the sphere obstacle.
    Parameters:
    ------------
    radius: float: Radius of the obstacle
    """
    radius: float

@dataclass
class SphereObstacleConfig(CollisionObstacleConfig):
    """
    Configuration dataclass for sphere obstacle.
    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    orientation: List: [a,b,c,d] Quaternion orientation of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    mass: float : Mass of the object, only used if movable set to true
    color : List : [r,g,b,a] rgba Color where r,g,b,a are floats between 0 and 1
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """
    geometry: GeometryConfig
    orientation: List[float] = field(default_factory=list)
    movable: bool = False
    mass: float = 1
    color: List[float] = field(default_factory=list)
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None

class SphereObstacle(CollisionObstacle):
    """
    Sphere obstacle class.
    """
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(SphereObstacleConfig)
        super().__init__(schema, **kwargs)
        self.add_required_keys({"geometry": ["radius"]})
        self.check_completeness()

    def limit_low(self):
        """
        Returns lowest limit.
        """
        if self._config.low:
            return [
                np.array(self._config.low.position),
                self._config.low.radius,
            ]
        else:
            return [np.ones(self.dimension()) * -1, 0]

    def limit_high(self):
        """
        Returns highest limit.
        """
        if self._config.high:
            return [
                np.array(self._config.high.position),
                self._config.high.radius,
            ]
        else:
            return [np.ones(self.dimension()) * 1, 1]

    def radius(self):
        """
        Return objecs radius.
        """
        return self._config.geometry.radius

    def shuffle(self):
        """
        Set new position and radius for object.
        """
        random_pos = np.random.uniform(
            self.limit_low()[0], self.limit_high()[0], self.dimension()
        )
        random_radius = np.random.uniform(
            self.limit_low()[1], self.limit_high()[1], 1
        )
        self._config.position = random_pos.tolist()
        self._config.geometry.radius = float(random_radius)

    def csv(self, file_name, samples=100):
        """
        Save as CSV file.
        """
        theta = np.arange(-np.pi, np.pi + np.pi / samples, step=np.pi / samples)
        x_pos = self.position()[0] + (self.radius() - 0.1) * np.cos(theta)
        y_pos = self.position()[1] + (self.radius() - 0.1) * np.sin(theta)
        with open(file_name, mode="w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            for i in range(2 * samples):
                csv_writer.writerow([x_pos[i], y_pos[i]])

    def render_gym(self, viewer, rendering):
        """
        Render object in gym environment.
        """
        if self.dimension() != 2:
            raise DimensionNotSuitableForEnv(
                "PlanarGym only supports two dimensional obstacles"
            )
        x_pos = self.position()
        joint = viewer.draw_circle(self.radius())
        joint.add_attr(rendering.Transform(
            rotation=0, translation=(x_pos[0], x_pos[1])))

    def add_to_bullet(self, pybullet):
        """
        Adds object to pybullet environment.
        """
        pybullet.setAdditionalSearchPath(
                os.path.dirname(os.path.realpath(__file__)))

        collision_shape = pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE, radius=self.radius())

        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="sphere.obj",
            rgbaColor=self.color(),
            meshScale=[self.radius(), self.radius(), self.radius()]
        )

        if self.dimension() == 2:
            base_position = self.position() + [0.0]
        elif self.dimension() == 3:
            base_position = self.position()
        else:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports three dimensional obstacles")

        base_orientation = self.orientation()

        mass = -1
        if self.movable():
            mass = self.mass()

        pybullet.createMultiBody(mass,
              collision_shape,
              visual_shape_id,
              base_position,
              base_orientation)
