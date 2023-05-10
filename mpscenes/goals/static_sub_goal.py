from typing import List, Optional, Any
from dataclasses import dataclass
import numpy as np
from pyquaternion import Quaternion
from omegaconf import OmegaConf

from mpscenes.goals.sub_goal import SubGoal, SubGoalConfig
from mpscenes.common.errors import DimensionNotSuitableForEnv


@dataclass
class StaticSubGoalConfig(SubGoalConfig):
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
    desired_position : list
        Goal state of the concerned link
    angle: list
        Additional rotation from the parent_link frame given by a quaternion
    low: list
        Lower limit for randomization
    high: list
        Upper limit for randomization

    """

    parent_link: Any
    child_link: Any
    desired_position: List[float]
    angle: Optional[Any] = None
    low: Optional[List[float]] = None
    high: Optional[List[float]] = None


class StaticSubGoal(SubGoal):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(StaticSubGoalConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        self.check_completeness()
        self.check_dimensionality()

    def parent_link(self):
        return self._config.parent_link

    def child_link(self):
        return self._config.child_link

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

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self._config.desired_position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs):
        return np.zeros(self.dimension())

    def shuffle(self):
        random_pos = np.random.uniform(
            self.limit_low(), self.limit_high(), self.dimension()
        )
        self._config.desired_position = random_pos.tolist()

    def angle(self):
        if isinstance(self._config.angle, float):
            return self._config.angle
        if self._config.angle:
            return list(self._config.angle)
