from dataclasses import dataclass
from typing import Optional, Any
import os
import numpy as np
import csv
from omegaconf import OmegaConf

from MotionPlanningSceneHelpers.analyticTrajectory import AnalyticTrajectory
from MotionPlanningSceneHelpers.splineTrajectory import SplineTrajectory
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    ComponentIncompleteError,
    DimensionNotSuitableForEnv,
)
from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)


class DynamicSphereObstacleMissmatchDimensionError(Exception):
    pass


class TypeNotSupportedError(Exception):
    pass


@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and radius of a dynamic sphere obstacle.

    Parameters:
    ------------

    trajectory: Any
        trajectory description of the obstacle. Can be either a spline or
        a analytic trajectory.
    radius: float
        radius of the obstacle
    """

    trajectory: Any
    radius: float


@dataclass
class DynamicSphereObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a dynamic spherical obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    low: GeometryConfig : Lower limit for randomization
    high: GeometryConfig : Upper limit for randomization
    """

    geometry: GeometryConfig
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None


class DynamicSphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(DynamicSphereObstacleConfig)
        super().__init__(schema, **kwargs)
        self.check_completeness()
        self.check_dimensionality()
        if self.type() == "splineSphere":
            self._traj = SplineTrajectory(
                self.dimension(), traj=self._config.geometry.trajectory
            )
        elif self.type() == "sphere" or self.type() == "analyticSphereObstacle":
            self._traj = AnalyticTrajectory(
                self.dimension(), traj=self._config.geometry.trajectory
            )
        elif self.type() == "analyticSphere":
            self._traj = AnalyticTrajectory(
                self.dimension(), traj=self._config.geometry.trajectory
            )
        self._traj.concretize()

    def check_dimensionality(self):
        if self.type() == "splineSphere":
            dim_verification = len(
                self.geometry()["trajectory"]["controlPoints"][0]
            )
        elif self.type() == "sphere" or self.type() == "analyticSphereObstacle":
            dim_verification = len(self.geometry()["trajectory"])
        elif self.type() == "analyticSphere":
            dim_verification = len(self.geometry()["trajectory"])
        else:
            raise TypeNotSupportedError(
                f"Obstacle type {self.type()} not supported"
            )
        if self.dimension() != dim_verification:
            raise DynamicSphereObstacleMissmatchDimensionError(
                "Dimension mismatch between trajectory array and dimension"
            )

    def dimension(self):
        if self.type() in ["sphere", "analyticSphere"]:
            return len(self._config.geometry.trajectory)
        elif self.type() in ["splineSphere"]:
            return len(self._config.geometry.trajectory.controlPoints[0])

    def traj(self):
        return self._traj

    def check_geometry_completeness(self):
        incomplete = False
        missing_keys = ""
        for key in self._geometry_keys:
            if key not in self.geometry():
                incomplete = True
                missing_keys += key + ", "
        if incomplete:
            raise ComponentIncompleteError(
                f"Missing keys in geometry: {missing_keys[:-2]}"
            )

    def position(self, **kwargs):
        if "t" not in kwargs:
            t = 0.0
        else:
            t = kwargs.get("t")
        return self._traj.evaluate(t)[0]

    def velocity(self, **kwargs):
        if "t" not in kwargs:
            t = 0.0
        else:
            t = kwargs.get("t")
        return self._traj.evaluate(t)[1]

    def acceleration(self, **kwargs):
        if "t" not in kwargs:
            t = 0.0
        else:
            t = kwargs.get("t")
        return self._traj.evaluate(t)[2]

    def radius(self):
        return self.geometry()["radius"]


    def movable(self):
        return False

    def csv(self, file_name, samples=100):
        theta = np.arange(-np.pi, np.pi, step=np.pi / samples)
        x = self.position()[0] + (self.radius() - 0.1) * np.cos(theta)
        y = self.position()[1] + (self.radius() - 0.1) * np.sin(theta)
        with open(file_name, mode="w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            for i in range(2 * samples):
                csv_writer.writerow([x[i], y[i]])

    def render_gym(self, viewer, rendering, **kwargs):
        x = self.position(t=kwargs.get("t"))
        if self.dimension() != 2:
            raise DimensionNotSuitableForEnv(
                "PlanarGym only supports two dimensional obstacles"
            )
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.radius())
        joint.add_attr(tf)

    def add_to_bullet(self, pybullet) -> int:
        if self.dimension() == 2:
            base_position= self.position().tolist() + [0.0]
        elif self.dimension() == 3:
            base_position= self.position().tolist()
        else:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports three dimensional obstacles"
            )
        collision_shape = pybullet.createCollisionShape(
            pybullet.GEOM_SPHERE,
            radius=self.radius(),
        )
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
        self._bullet_id = pybullet.createMultiBody(
            mass, collision_shape, visual_shape_id, base_position, base_orientation
        )
        return self._bullet_id

    def update_bullet_position(self, pybullet, **kwargs):
        if "t" not in kwargs:
            t = 0.0
        else:
            t = kwargs.get("t")
        pos = self.position(t=t).tolist()
        if self.dimension() == 2:
            pos += [0.0]
        ori = [0, 0, 0, 1]
        pybullet.resetBasePositionAndOrientation(self._bullet_id, pos, ori)
