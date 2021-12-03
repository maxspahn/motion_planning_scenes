import os

from MotionPlanningEnv.collisionObstacle import CollisionObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv


class UrdfObstacle(CollisionObstacle):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.checkCompleteness()
        self.checkUrdfFile()
        self.checkGeometryCompleteness()

    def checkUrdfFile(self):
        if 'urdf' not in self._contentDict:
            raise ComponentIncompleteError("Missing urdf file")

    def urdf(self):
        return self._contentDict['urdf']

    def checkGeometryCompleteness(self):
        if 'position' not in self.geometry():
            raise ComponentIncompleteError("Missing position in geometry for urdf obstacle")

    def position(self):
        return self.geometry()['position']

    def toDict(self):
        return self._contentDict

    def add2Bullet(self, pybullet):
        if self.dim() != 3:
            raise DimensionNotSuitableForEnv("Pybullet only supports two dimensional obstacles")
        pybullet.loadURDF(fileName=self.urdf(), basePosition=self.position())
