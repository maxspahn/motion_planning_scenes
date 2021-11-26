from MotionPlanningGoal.subGoal import SubGoal


class StaticSubGoal(SubGoal):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.checkCompleteness()
        self.checkDimensionality()

    def type(self):
        return 'static'

    def toDict(self):
        return {}

    def getDesiredPosition(self):
        return self._contentDict['desired_position']

    def evaluate(self, t):
        return self._contentDict['desired_position']

    def generateRandomPosition(self):
        self._pos = [0.0, 0.0]
