from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle


class UnknownObstacleType(Exception):
    pass


class ObstacleCreator(object):
    def __init__(self):
        pass

    def createObstacle(self, obstacleType, name, content_dict):
        if obstacleType == "sphereObstacle":
            return SphereObstacle(name=name, content_dict=content_dict)
        elif obstacleType == "analyticSphere":
            return DynamicSphereObstacle(name=name, content_dict=content_dict)
        elif obstacleType == "splineSphere":
            return DynamicSphereObstacle(name=name, content_dict=content_dict)
        else:
            raise UnknownObstacleType("ObstacleType %s is not know" % obstacleType)
