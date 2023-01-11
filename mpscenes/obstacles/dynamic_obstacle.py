from dataclasses import dataclass
from typing import Optional, Any
import os
import numpy as np
import csv
from omegaconf import OmegaConf

from mpscenes.obstacles.collision_obstacle import CollisionObstacle, CollisionObstacleConfig
from mpscenes.common.errors import MissmatchDimensionError, TrajectoryNotSupported
from mpscenes.common.analytic_trajectory import AnalyticTrajectory
from mpscenes.common.spline_trajectory import SplineTrajectory
from mpscenes.common.component import MPComponent

@dataclass
class DynamicGeometryConfig:
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

class DynamicObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = kwargs.get('schema')
        super().__init__(**kwargs)
        if "controlPoints" in self.geometry()['trajectory']:
            self._trajectory_type = "spline"
        elif isinstance(self.geometry()['trajectory'][0], str):
            self._trajectory_type = "analytic"
        else:
            raise TrajectoryNotSupported(
                f"Trajectory definition not supported, {self.geometry()['trajectory']}"
            )
        self.check_completeness()
        self.check_dimensionality()
        if self.trajectory_type() == 'spline':
            self._traj = SplineTrajectory(
                self.dimension(), traj=self._config.geometry.trajectory
            )
        elif self.trajectory_type() == "analytic":
            self._traj = AnalyticTrajectory(
                self.dimension(), traj=self._config.geometry.trajectory
            )
        self._traj.concretize()

    def trajectory_type(self) -> str:
        return self._trajectory_type

    def check_dimensionality(self):
        dim_verification = self.dimension()
        if self.dimension() != dim_verification:
            raise MissmatchDimensionError(
                "Dynamic Obstacle: Dimension mismatch between trajectory array and dimension"
            )

    def dimension(self) -> int:
        if self.trajectory_type() == 'spline':
            return len(
                self.geometry()["trajectory"]["controlPoints"][0]
            )
        if self.trajectory_type() == 'analytic':
            return len(self.geometry()["trajectory"])

    def traj(self):
        return self._traj

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
