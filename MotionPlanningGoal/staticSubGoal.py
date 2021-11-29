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

    def position(self, **kwargs):
        return self._contentDict['desired_position']

    def shuffle(self):
        self._pos = [0.0, 0.0]

    def renderGym(self, viewer):
        from gym.envs.classic_control import rendering
        x = self.position()
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(0.1, color=[0.1, 0.1, 0.1])
        joint.add_attr(tf)
