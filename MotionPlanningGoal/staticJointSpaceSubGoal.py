import numpy as np
from MotionPlanningGoal.subGoal import SubGoal
from MotionPlanningSceneHelpers.motionPlanningComponent import DimensionNotSuitableForEnv

class JointSpaceGoalsNotSupportedError(Exception):
    pass


class StaticJointSpaceSubGoal(SubGoal):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._required_keys = [
            "m",
            "w",
            "prime",
            "epsilon",
            "type",
            "desired_position",
            "indices",
        ]
        self.checkCompleteness()
        self.checkDimensionality()

    def limitLow(self):
        if 'low' in self._contentDict:
            return np.array(self._contentDict['low'])
        else:
            return np.ones(self.m()) * -1

    def limitHigh(self):
        if 'high' in self._contentDict:
            return np.array(self._contentDict['high'])
        else:
            return np.ones(self.m()) * 1

    def evaluate(self, **kwargs):
        return []

    def toDict(self):
        return self._contentDict

    def position(self, **kwargs):
        return self._contentDict['desired_position']

    def velocity(self, **kwargs):
        return np.zeros(m)

    def acceleration(self, **kwargs):
        return np.zeros(m)

    def shuffle(self):
        randomPos = np.random.uniform(self.limitLow(), self.limitHigh(), self.m())
        self._contentDict['desired_position'] = randomPos.tolist()

    def renderGym(self, viewer, **kwargs):
        raise JointSpaceGoalsNotSupportedError()

    def add2Bullet(self, pybullet, **kwargs):
        raise JointSpaceGoalsNotSupportedError()
