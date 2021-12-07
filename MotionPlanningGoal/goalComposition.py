from abc import ABC, abstractmethod
from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent
from MotionPlanningGoal.subGoalCreator import SubGoalCreator


class MultiplePrimeGoalsError(Exception):
    pass


class GoalComposition(MotionPlanningComponent):
    def __init__(self, **kwargs):
        self._required_keys = [
            "subgoal0",
        ]
        super().__init__(**kwargs)
        self._primeGoalIndex = -1
        self._subGoals = []
        self._subGoalCreator = SubGoalCreator()
        self.parseSubGoals()

    def parseSubGoals(self):
        for subGoalName in self._contentDict.keys():
            subGoalType = self._contentDict[subGoalName]['type']
            subGoalDict = self._contentDict[subGoalName]
            subGoal = self._subGoalCreator.createSubGoal(subGoalType, subGoalName, subGoalDict)
            if subGoal.isPrimeGoal():
                if self._primeGoalIndex >= 0:
                    raise MultiplePrimeGoalsError("There are multiple prime goals. Using first prime goal")
                else:
                    self._primeGoalIndex = len(self._subGoals)
            self._subGoals.append(subGoal)

    def primeGoal(self):
        return self.getGoalByIndex(self._primeGoalIndex)

    def subGoals(self):
        return self._subGoals

    def getGoalByName(self, name):
        for subGoal in self._subGoals:
            if subGoal.name() == name:
                return subGoal

    def getGoalByIndex(self, index):
        return self._subGoals[index]

    def evaluate(self, **kwargs):
        evals = []
        for subGoal in self._subGoals:
            evals += subGoal.evaluate(**kwargs)
        return evals

    def toDict(self):
        compositionDict = {}
        for subGoal in self._subGoals:
            compositionDict[subGoal.name()] = subGoal.toDict()
        return compositionDict

    def shuffle(self):
        for subGoal in self._subGoals:
            subGoal.shuffle()

    def renderGym(self, viewer, **kwargs):
        self.primeGoal().renderGym(viewer, **kwargs)

    def add2Bullet(self, pybullet):
        self.primeGoal().add2Bullet(pybullet)

    def updateBulletPosition(self, pybullet, **kwargs):
        self.primeGoal().updateBulletPosition(pybullet, **kwargs)

