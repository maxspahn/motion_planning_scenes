from abc import ABC, abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent


class SubGoalMissmatchDimensionError(Exception):
    pass


class SubGoal(MotionPlanningComponent):
    def __init__(self, **kwargs):
        self._required_keys = [
            "m",
            "w",
            "prime",
            "indices",
            "parent_link",
            "child_link",
            "epsilon",
            "type",
        ]
        super().__init__(**kwargs)

    def checkDimensionality(self):
        if self.m() != len(self.position()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between goal and m"
            )
        if self.m() != len(self.indices()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between indices and m"
            )

    def isPrimeGoal(self):
        return self._contentDict["prime"]

    def epsilon(self):
        return self._contentDict['epsilon']

    def indices(self):
        return self._contentDict["indices"]

    def m(self):
        return self._contentDict["m"]

    def parentLink(self):
        return self._contentDict["parent_link"]

    def childLink(self):
        return self._contentDict["child_link"]

    def weight(self):
        return self._contentDict["w"]

    def type(self):
        return self._contentDict['type']

    def updateBulletPosition(self, pybullet, **kwargs):
        pass

    @abstractmethod
    def position(self, **kwargs):
        pass

    @abstractmethod
    def shuffle(self):
        pass

