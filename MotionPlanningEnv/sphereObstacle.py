import numpy as np
from copy import deepcopy
from MotionPlanningEnv.collisionObstacle import CollisionObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv


class SphereObstacleMissmatchDimensionError(Exception):
    pass


class SphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['position', 'radius']
        self.checkCompleteness()
        self.checkGeometryCompleteness()
        self._position = [float(val) for val in self._contentDict['geometry']['position']]
        self._radius = float(self._contentDict['geometry']['radius'])
        self.checkDimensionality()

    def checkDimensionality(self):
        if self.dim() != len(self.position()):
            raise SphereObstacleMissmatchDimensionError(
                "Dimension mismatch between position array and dimension"
            )

    def checkGeometryCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._geometry_keys:
            if key not in self.geometry():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise ComponentIncompleteError("Missing keys in geometry: %s" % missingKeys[:-2])

    def limitLow(self):
        if 'low' in self._contentDict:
            return [np.array(self._contentDict['low']['position']), float(self._contentDict['low']['radius'])]
        else:
            return [np.ones(self.dim()) * -1, 0]

    def limitHigh(self):
        if 'high' in self._contentDict:
            return [np.array(self._contentDict['high']['position']), float(self._contentDict['high']['radius'])]
        else:
            return [np.ones(self.dim()) * 1, 1]

    def position(self, **kwargs):
        return self._position

    def radius(self):
        return self._radius

    def toDict(self):
        contentDict = deepcopy(self._contentDict)
        contentDict['geometry']['position'] = self._position
        contentDict['geometry']['radius'] = self._radius
        return contentDict

    def shuffle(self):
        randomPos = np.random.uniform(self.limitLow()[0], self.limitHigh()[0], self.dim())
        randomRadius = np.random.uniform(self.limitLow()[1], self.limitHigh()[1], 1)
        self._position = randomPos.tolist()
        self._radius = float(randomRadius)

    def movable(self):
        if 'movable' in self._contentDict:
            return self._contentDict['movable']
        else:
            return False

    def toCSV(self, fileName, samples=100):
        import numpy as np
        import csv
        theta = np.arange(-np.pi, np.pi, step=np.pi/samples)
        x = self.position()[0] + (self.radius()-0.1) * np.cos(theta)
        y = self.position()[1] + (self.radius()-0.1) * np.sin(theta)
        with open(fileName, mode='w') as file:
            csv_writer = csv.writer(file, delimiter=',')
            for i in range(2*samples):
                csv_writer.writerow([x[i], y[i]])

    def renderGym(self, viewer, **kwargs):
        from gym.envs.classic_control import rendering
        if self.dim() != 2:
            raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")
        x = self.position()
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.radius())
        joint.add_attr(tf)

    def add2Bullet(self, pybullet):
        if self.dim() == 2:
            basePosition = self.position() + [0.0]
        elif self.dim() == 3:
            basePosition = self.position()
        else:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        collisionShape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=self.radius())
        visualShapeId = -1
        baseOrientation = [0, 0, 0, 1]
        mass = int(self.movable())
        visualShapeId = -1

        pybullet.createMultiBody(mass,
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)
