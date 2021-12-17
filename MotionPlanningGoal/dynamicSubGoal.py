from MotionPlanningGoal.subGoal import SubGoal
from MotionPlanningSceneHelpers.motionPlanningComponent import DimensionNotSuitableForEnv
from MotionPlanningSceneHelpers.analyticTrajectory import AnalyticTrajectory
from MotionPlanningSceneHelpers.splineTrajectory import SplineTrajectory


class DynamicSubGoal(SubGoal):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.checkCompleteness()
        if self.type() == 'splineSubGoal':
            self._traj = SplineTrajectory(self.m(), traj=self._contentDict['trajectory'])
        elif self.type() == 'analyticSubGoal':
            self._traj = AnalyticTrajectory(self.m(), traj=self._contentDict['trajectory'])
        self._traj.concretize()
        self.checkDimensionality()

    def traj(self):
        return self._traj

    def toDict(self):
        return self._contentDict

    def position(self, **kwargs):
        return self.evaluate(**kwargs)[0]

    def evaluate(self, **kwargs):
        t = kwargs.get('t') if 't' in kwargs else 0.0
        return self._traj.evaluate(t=t)

    def shuffle(self):
        pass

    def renderGym(self, viewer, **kwargs):
        from gym.envs.classic_control import rendering
        if self.m() != 2:
            raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")
        x = self.position(t=kwargs.get('t'))
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
        joint.add_attr(tf)

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
        basePosition = self.position()
        baseOrientation = [0, 0, 0, 1]
        mass = 0

        self._bulletId = pybullet.createMultiBody(
                    mass,
                    collisionShape,
                    visualShapeId,
                    basePosition,
                    baseOrientation,
        )

    def updateBulletPosition(self, pybullet, **kwargs):
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        pos = self.position(t=t).tolist()
        if self.m() == 2 and self.indices() == [0, 1]:
            pos += [0.0]
        ori = [0, 0, 0, 1]
        pybullet.resetBasePositionAndOrientation(self._bulletId, pos, ori)
