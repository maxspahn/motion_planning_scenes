from typing import List, Optional
from dataclasses import dataclass
import numpy as np
from omegaconf import OmegaConf

from mpscenes.goals.sub_goal import SubGoal, SubGoalConfig
from mpscenes.common.errors import JointSpaceGoalsNotSupportedError


@dataclass
class StaticJointSpaceSubGoalConfig(SubGoalConfig):
    """Configuration dataclass for static joint space sub goal.

    This configuration class holds information about the
    the desired joint configuration and the limits for randomization.

    Parameters:
    ------------

    desired_position : list : Goal configuration of the robot
    low : list : Lower limit for randomization
    high : list : Upper limit for randomization
    """

    desired_position: List[float]
    low: Optional[List[float]] = None
    high: Optional[List[float]] = None


class StaticJointSpaceSubGoal(SubGoal):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(StaticJointSpaceSubGoalConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        self.check_completeness()
        self.check_dimensionality()

    def limit_low(self):
        if self._config.low:
            return np.array(self._config.low)
        else:
            return np.ones(self.dimension()) * -1

    def limit_high(self):
        if self._config.high:
            return np.array(self._config.high)
        else:
            return np.ones(self.dimension()) * 1

    def evaluate(self, **kwargs):
        pass

    def position(self, **kwargs):
        return self._config.desired_position

    def velocity(self, **kwargs):
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs):
        return np.zeros(self.dimension())

    def shuffle(self):
        random_pos = np.random.uniform(
            self.limit_low(), self.limit_high(), self.dimension()
        )
        self._config.desired_position = random_pos.tolist()

    def render_gym(self, viewer, **kwargs):
        raise JointSpaceGoalsNotSupportedError()

    def add_to_bullet(self, pybullet, **kwargs):
        raise JointSpaceGoalsNotSupportedError()
