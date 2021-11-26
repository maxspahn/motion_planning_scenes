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
            "desired_position",
        ]
        super().__init__(**kwargs)

    def checkDimensionality(self):
        if self.m() != len(self.getDesiredPosition()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between goal and m"
            )
        if self.m() != len(self.indices()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between indices and m"
            )

    def isPrimeGoal(self):
        return self._contentDict["prime"]

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

    @abstractmethod
    def type(self):
        pass

    @abstractmethod
    def getDesiredPosition(self):
        pass

    @abstractmethod
    def evaluate(self, t):
        pass

    @abstractmethod
    def generateRandomPosition(self):
        pass

