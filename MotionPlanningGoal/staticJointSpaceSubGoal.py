import numpy as np
from MotionPlanningGoal.subGoal import SubGoal, SubGoalConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import DimensionNotSuitableForEnv

from omegaconf import OmegaConf
from typing import List, Optional
from dataclasses import dataclass

class JointSpaceGoalsNotSupportedError(Exception):
    pass

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
        super().__init__(**kwargs)
        schema = OmegaConf.structured(StaticJointSpaceSubGoalConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        self.checkDimensionality()

    def limitLow(self):
        if self._config.low:
            return np.array(self._config.low)
        else:
            return np.ones(self.m()) * -1

    def limitHigh(self):
        if self._config.high:
            return np.array(self._config.high)
        else:
            return np.ones(self.m()) * 1

    def evaluate(self, **kwargs):
        return []

    def toDict(self):
        return OmegaConf.to_container(self._config)

    def position(self, **kwargs):
        return self._config.desired_position

    def velocity(self, **kwargs):
        return np.zeros(self.m())

    def acceleration(self, **kwargs):
        return np.zeros(self.m())

    def shuffle(self):
        randomPos = np.random.uniform(self.limitLow(), self.limitHigh(), self.m())
        self._config.desired_position = randomPos.tolist()

    def renderGym(self, viewer, **kwargs):
        raise JointSpaceGoalsNotSupportedError()

    def add2Bullet(self, pybullet, **kwargs):
        raise JointSpaceGoalsNotSupportedError()
