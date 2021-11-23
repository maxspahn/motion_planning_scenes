from MotionPlanningEnv.collisionObstacle import CollisionObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError

class UrdfObstacle(CollisionObstacle):
    def __init__(self, name, obstDict):
        super().__init__(name, obstDict)
        self.checkCompleteness()
        self.checkUrdfFile()
        self.checkGeometryCompleteness()

    def checkUrdfFile(self):
        if 'urdf' not in self._contentDict:
            raise ComponentIncompleteError("Missing urdf file")

    def urdf(self):
        return self._contentDict['urdf']

    def checkGeometryCompleteness(self):
        incomplete = False
        missingKeys = ""
        if 'position' not in self.geometry():
            raise ComponentIncompleteError("Missing position in geometry for urdf obstacle")

    def position(self):
        return self.geometry()['position']

    def toDict(self):
        return self._contentDict
