from dataclasses import dataclass
from typing import List, Optional, Dict, Any
import numpy as np
import os
from copy import deepcopy
from omegaconf import OmegaConf

from MotionPlanningSceneHelpers.analyticTrajectory import AnalyticTrajectory
from MotionPlanningSceneHelpers.splineTrajectory import SplineTrajectory
from MotionPlanningEnv.collisionObstacle import CollisionObstacle, CollisionObstacleConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv



class DynamicSphereObstacleMissmatchDimensionError(Exception):
    pass


class TypeNotSupportedError(Exception):
    pass

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and radius of a dynamic sphere obstacle.

    Parameters:
    ------------

    trajectory: Any: Trajectory description of the obstacle. Can be either a spline or
        a analytic trajectory.
    radius: float: Radius of the obstacle
    """
    trajectory: Any
    radius: float


@dataclass
class DynamicSphereObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size 
    and randomization of a dynamic spherical obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    low: GeometryConfig : Lower limit for randomization
    high: GeometryConfig : Upper limit for randomization
    """
    geometry: GeometryConfig
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None


class DynamicSphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['trajectory', 'radius']
        schema = OmegaConf.structured(DynamicSphereObstacleConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        self.checkDimensionality()
        self.checkGeometryCompleteness()
        if self.type() == 'splineSphere':
            self._traj = SplineTrajectory(self.dim(), traj=self._config.geometry.trajectory)
        elif self.type() == 'sphere' or self.type() == 'analyticSphereObstacle':
            self._traj = AnalyticTrajectory(self.dim(), traj=self._config.geometry.trajectory)
        elif self.type() == 'analyticSphere':
            self._traj = AnalyticTrajectory(self.dim(), traj=self._config.geometry.trajectory)
        self._traj.concretize()

    def checkDimensionality(self):
        if self.type() == 'splineSphere':
            dim_verification = len(self.geometry()['trajectory']['controlPoints'][0])
        elif self.type() == 'sphere' or self.type() == 'analyticSphereObstacle':
            dim_verification = len(self.geometry()['trajectory'])
        elif self.type() == 'analyticSphere':
            dim_verification = len(self.geometry()['trajectory'])
        else:
            raise TypeNotSupportedError(f"Obstacle type {self.type()} not supported")
        if self.dim() != dim_verification:
            raise DynamicSphereObstacleMissmatchDimensionError(
                "Dimension mismatch between trajectory array and dimension"
            )

    def traj(self):
        return self._traj

    def checkGeometryCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._geometry_keys:
            if key not in self.geometry():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise ComponentIncompleteError("Missing keys in geometry: %s" % missingKeys[:-2])

    def position(self, **kwargs):
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        return self._traj.evaluate(t)[0]

    def velocity(self, **kwargs):
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        return self._traj.evaluate(t)[1]
        
    def acceleration(self, **kwargs):
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        return self._traj.evaluate(t)[2]

    def radius(self):
        return self.geometry()['radius']

    def toDict(self):
        return OmegaConf.to_container(self._config)

    def movable(self):
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

    def renderGym(self, viewer, rendering, **kwargs):
        x = self.position(t=kwargs.get('t'))
        if self.dim() != 2:
            raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")
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
        collisionShape = pybullet.createCollisionShape(
            pybullet.GEOM_SPHERE, 
            radius=self.radius(),
        )
        basePosition = self.position()
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

        self._bulletId = pybullet.createMultiBody(mass,
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)

    def updateBulletPosition(self, pybullet, **kwargs):
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        pos = self.position(t=t).tolist()
        if self.dim() == 2:
            pos += [0.0]
        ori = [0, 0, 0, 1]
        pybullet.resetBasePositionAndOrientation(self._bulletId, pos, ori)
