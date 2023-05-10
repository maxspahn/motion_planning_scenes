from dataclasses import dataclass
from typing import List, Optional, Any
import numpy as np
from omegaconf import OmegaConf
from pyquaternion import Quaternion
from mpscenes.goals.sub_goal import SubGoal, SubGoalConfig
from mpscenes.common.errors import DimensionNotSuitableForEnv, TrajectoryNotSupported
from mpscenes.common.analytic_trajectory import AnalyticTrajectory
from mpscenes.common.spline_trajectory import SplineTrajectory


@dataclass
class DynamicSubGoalConfig(SubGoalConfig):
    """Configuration dataclass for static sub goal.

    This configuration class holds information about the
    the weight, accuracy required, type and position in the
    kinematic chain.

    Parameters:
    ------------

    parent_link: str
        Name of the link that specifies the frame in which the goal is defined
    child_link: str
        Name of the link that should match the desired position
    trajectory: Any
        Trajectory of the goal, either defined by spline or analytic equation.
    angle list
        Additional rotation from the parent_link frame given by a quaternion
    low : list
        Lower limit for randomization
    high : list
        Upper limit for randomization

    """

    parent_link: str
    child_link: str
    trajectory: Any
    angle: Optional[Any] = None
    low: Optional[List[float]] = None
    high: Optional[List[float]] = None


class DynamicSubGoal(SubGoal):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(DynamicSubGoalConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        if "controlPoints" in self._config.trajectory:
            self._trajectory_type = "spline"
        elif isinstance(self._config.trajectory[0], str):
            self._trajectory_type = "analytic"
        else:
            raise TrajectoryNotSupported(
                f"Trajectory definition not supported, {self._config.trajectory}"
            )
        self.check_completeness()
        if self.trajectory_type() == 'spline':
            self._traj = SplineTrajectory(
                self.dimension(), traj=self._config.trajectory
            )
        elif self.trajectory_type() == "analytic":
            self._traj = AnalyticTrajectory(
                self.dimension(), traj=self._config.trajectory
            )
        self._traj.concretize()
        self.check_dimensionality()

    def trajectory_type(self) -> str:
        return self._trajectory_type

    def parent_link(self):
        return self._config.parent_link

    def child_link(self):
        return self._config.child_link

    def traj(self):
        return self._traj

    def dict(self):
        return OmegaConf.to_container(self._config)

    def position(self, **kwargs) -> np.ndarray:
        return self.evaluate_trajectory(**kwargs)[0]

    def velocity(self, **kwargs) -> np.ndarray:
        return self.evaluate_trajectory(**kwargs)[1]

    def acceleration(self, **kwargs) -> np.ndarray:
        return self.evaluate_trajectory(**kwargs)[2]

    def evaluate_trajectory(self, **kwargs):
        t = kwargs.get("t") if "t" in kwargs else 0.0
        return self._traj.evaluate(time_step=t)

    def shuffle(self):
        self._traj.shuffle()

    def angle(self):
        return self._config.angle
