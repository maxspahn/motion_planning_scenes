from abc import abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    MotionPlanningComponent,
)
from dataclasses import dataclass

from typing import List


class SubGoalMissmatchDimensionError(Exception):
    pass


@dataclass
class SubGoalConfig:
    """Configuration dataclass for sub goal.

    This configuration class holds information about the
    the weight, accuracy required, type and position in the
    kinematic chain.

    Parameters:
    ------------

    m: int: Dimension of the sub goal
    w: float: Weight of the sub goal
    type: str: Type of the sub goal
    indices: list: Indices of a forward map to be considered
    epsilon: float: Required accuracy of the sub goal
    prime: bool: Flag for primary goal
    """

    weight: float
    type: str
    indices: List[int]
    epsilon: float
    is_primary_goal: bool


class SubGoal(MotionPlanningComponent):
    def check_dimensionality(self):
        if isinstance(self.position(), str):
            return
        if len(self.indices()) != len(self.position()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between goal and indices"
            )

    def is_primary_goal(self):
        return self._config.is_primary_goal

    def epsilon(self):
        return self._config.epsilon

    def indices(self):
        return self._config.indices

    def dimension(self):
        return len(self.indices())

    def weight(self):
        return self._config.weight

    def type(self):
        return self._config.type

    def update_bullet_position(self, pybullet, **kwargs):
        pass

    def evaluate(self, **kwargs) -> list:
        return [
            self.position(**kwargs),
            self.velocity(**kwargs),
            self.acceleration(**kwargs)
        ]

    @abstractmethod
    def position(self, **kwargs):
        pass

    @abstractmethod
    def velocity(self, **kwargs):
        pass

    @abstractmethod
    def acceleration(self, **kwargs):
        pass

    @abstractmethod
    def shuffle(self):
        pass
