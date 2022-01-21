from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.staticJointSpaceSubGoal import StaticJointSpaceSubGoal
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal


class UnknownSubGoalType(Exception):
    pass


class SubGoalCreator(object):
    def __init__(self):
        pass

    def createSubGoal(self, subGoalType, name, contentDict):
        if subGoalType == 'staticSubGoal':
            return StaticSubGoal(name=name, contentDict=contentDict)
        elif subGoalType == 'analyticSubGoal' or subGoalType == 'splineSubGoal':
            return DynamicSubGoal(name=name, contentDict=contentDict)
        elif subGoalType == 'staticJointSpaceSubGoal':
            return StaticJointSpaceSubGoal(name=name, contentDict=contentDict)
        else:
            raise UnknownSubGoalType("SubGoalType %s is not known" % subGoalType)
