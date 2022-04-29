import numpy as np
from MotionPlanningGoal.subGoal import SubGoal
from MotionPlanningSceneHelpers.motionPlanningComponent import DimensionNotSuitableForEnv


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

    def evaluate(self, **kwargs):
        return []

    def toDict(self):
        return self._contentDict

    def position(self, **kwargs):
        return self._contentDict['desired_position']

    def velocity(self, **kwargs):
        return np.zeros(self.dim())

    def acceleration(self, **kwargs):
        return np.zeros(self.dim())

    def shuffle(self):
        randomPos = np.random.uniform(self.limitLow(), self.limitHigh(), self.m())
        self._contentDict['desired_position'] = randomPos.tolist()

    def renderGym(self, viewer, rendering, **kwargs):
        if self.m() == 1:
            start_point = [-5.0, 5.0]
            end_point = [5.0, 5.0]
            start_point[self.indices()] = self.position()
            end_point[self.indices()] = self.position()
            line = viewer.draw_line(start_position, end_position)
        elif self.m() == 2:
            x = self.position()
            tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
            joint = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
            joint.add_attr(tf)
        else:
            raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")


    def add2Bullet(self, pybullet):
        if self.m() == 2 and self.indices() == [0, 1]:
            basePosition = self.position() + [0.0]
        elif self.m() == 3:
            basePosition = self.position()
        else:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        rgbaColor = [0.0, 1.0, 0.0, 0.3]
        visualShapeId = pybullet.createVisualShape(pybullet.GEOM_SPHERE, rgbaColor=rgbaColor, radius=self.epsilon())
        collisionShape = -1
        baseOrientation = [0, 0, 0, 1]
        mass = 0

        pybullet.createMultiBody(
                    mass,
                    collisionShape,
                    visualShapeId,
                    basePosition,
                    baseOrientation,
        )
