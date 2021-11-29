from MotionPlanningGoal.staticSubGoal import StaticSubGoal


class UnknownSubGoalType(Exception):
    pass


class SubGoalCreator(object):
    def __init__(self):
        pass

    def createSubGoal(self, subGoalType, name, contentDict):
        if subGoalType == 'staticSubGoal':
            return StaticSubGoal(name=name, contentDict=contentDict)
        else:
            raise UnknownSubGoalType("SubGoalType %s is not known" % subGoalType)
