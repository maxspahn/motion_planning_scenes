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

    def angle(self):
        if 'angle' in self._contentDict:
            return float(self._contentDict['angle'])
        else:
            return 0.0

    def renderGym(self, viewer, rendering, **kwargs):
        coordinate_system_1 = viewer.draw_line([-3, 0], [3, 0], color=[0.0, 0.0, 0.0])
        coordinate_system_2 = viewer.draw_line([0, -3], [0, 3], color=[0.0, 0.0, 0.0])
        tf2 = rendering.Transform(rotation=self.angle())
        coordinate_system_1.add_attr(tf2)
        coordinate_system_2.add_attr(tf2)
        if self.m() == 1:
            start_point = [-10, -10.0]
            end_point = [10.0, 10.0]
            start_point[self.indices()[0]] = 0.0
            end_point[self.indices()[0]] = 0.0
            goal = viewer.draw_line(start_point, end_point, color=[0.0, 1.0, 0.0])
            translation = [0.0, 0.0]
            translation[self.indices()[0]] = self.position()[0]
            tf = rendering.Transform(translation=translation)
            goal.add_attr(tf)
            goal.add_attr(tf2)
        elif self.m() == 2:
            x = self.position()
            goal = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
            tf = rendering.Transform(translation=(x[0], x[1]))
            goal.add_attr(tf)
            goal.add_attr(tf2)
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
