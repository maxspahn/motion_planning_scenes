import numpy as np
from MotionPlanningGoal.subGoal import SubGoal
from MotionPlanningSceneHelpers.motionPlanningComponent import DimensionNotSuitableForBullet


class StaticSubGoal(SubGoal):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
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

    def type(self):
        return self._contentDict['type']

    def toDict(self):
        return self._contentDict

    def position(self, **kwargs):
        return self._contentDict['desired_position']

    def shuffle(self):
        randomPos = np.random.uniform(self.limitLow(), self.limitHigh(), self.m())
        self._contentDict['desired_position'] = randomPos.tolist()

    def renderGym(self, viewer):
        from gym.envs.classic_control import rendering
        x = self.position()
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
        joint.add_attr(tf)

    def add2Bullet(self, pybullet):
        if self.m() != 3:
            raise DimensionNotSuitableForBullet("Pybullet only supports three dimensional obstacles")
        rgbaColor = [0.0, 1.0, 0.0, 1.0]
        visualShapeId = pybullet.createVisualShape(pybullet.GEOM_SPHERE, rgbaColor=rgbaColor, radius=self.epsilon())
        collisionShape = -1
        basePosition = self.position()
        baseOrientation = [0, 0, 0, 1]
        mass = 0

        pybullet.createMultiBody(
                    mass,
                    collisionShape,
                    visualShapeId,
                    basePosition,
                    baseOrientation,
        )
