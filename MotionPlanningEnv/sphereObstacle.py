from dataclasses import dataclass
import numpy as np
import os
from copy import deepcopy
from MotionPlanningEnv.collisionObstacle import CollisionObstacle, CollisionObstacleConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv

from omegaconf import OmegaConf
from typing import List, Optional, Dict

class SphereObstacleMissmatchDimensionError(Exception):
    pass

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and radius of a sphere obstacle.

    Parameters:
    ------------

    position: list: Position of the obstacle
    radius: float: Radius of the obstacle
    """
    position: List[float]
    radius: float


@dataclass
class SphereObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size 
    and randomization of a spherical obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """
    geometry: GeometryConfig
    movable: bool = False
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None

class SphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['position', 'radius']
        schema = OmegaConf.structured(SphereObstacleConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        self.checkGeometryCompleteness()
        self._radius = self._config.geometry.radius
        self._position = self._config.geometry.position
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
        if self._config.low:
            return [np.array(self._config.low.position), self._config.low.radius]
        else:
            return [np.ones(self.dim()) * -1, 0]

    def limitHigh(self):
        if self._config.high:
            return [np.array(self._config.high.position), self._config.high.radius]
        else:
            return [np.ones(self.dim()) * 1, 1]

    def position(self, **kwargs):
        return self._position

    def velocity(self, **kwargs):
        return np.zeros(self.dim())

    def acceleration(self, **kwargs):
        return np.zeros(self.dim())

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
        return self._config.movable

    def toCSV(self, fileName, samples=100):
        import numpy as np
        import csv
        theta = np.arange(-np.pi, np.pi + np.pi/samples, step=np.pi/samples)
        x = self.position()[0] + (self.radius()-0.1) * np.cos(theta)
        y = self.position()[1] + (self.radius()-0.1) * np.sin(theta)
        with open(fileName, mode='w') as file:
            csv_writer = csv.writer(file, delimiter=',')
            for i in range(2*samples):
                csv_writer.writerow([x[i], y[i]])

    def renderGym(self, viewer, rendering, **kwargs):
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
        pybullet.setAdditionalSearchPath(os.path.dirname(os.path.realpath(__file__)))
        visualShapeId = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName='sphere_smooth.obj',
            rgbaColor=[1.0, 0.0, 0.0, 1.0],
            specularColor=[1.0, 0.5, 0.5],
            meshScale=[self.radius(), self.radius(), self.radius()]
        )
        pybullet.createMultiBody(mass,
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)
