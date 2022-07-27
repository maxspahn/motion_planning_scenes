from abc import abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent
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
    m: int
    w: float
    type: str
    indices: List[int]
    epsilon: float
    prime: bool



class SubGoal(MotionPlanningComponent):
    def __init__(self, **kwargs):
        self._required_keys = [
            "m",
            "w",
            "prime",
            "indices",
            "epsilon",
            "type",
        ]
        super().__init__(**kwargs)

    def checkDimensionality(self):
        if isinstance(self.position(), str):
            return
        if self.m() != len(self.position()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between goal and m"
            )
        if self.m() != len(self.indices()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between indices and m"
            )

    def isPrimeGoal(self):
        return self._config.prime

    def epsilon(self):
        return self._config.epsilon

    def indices(self):
        return self._config.indices

    def m(self):
        return self._config.m

    def weight(self):
        return self._config.w

    def type(self):
        return self._config.type

    def updateBulletPosition(self, pybullet, **kwargs):
        pass

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

