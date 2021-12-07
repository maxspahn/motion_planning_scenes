from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle


class UnknownObstacleType(Exception):
    pass


class ObstacleCreator(object):
    def __init__(self):
        pass

    def createObstacle(self, obstacleType, name, contentDict):
        if obstacleType == "sphereObstacle":
            return SphereObstacle(name=name, contentDict=contentDict)
        elif obstacleType == "dynamicSphereObstacle":
            return DynamicSphereObstacle(name=name, contentDict=contentDict)
        else:
            raise UnknownObstacleType("ObstacleType %s is not know" % obstacleType)
