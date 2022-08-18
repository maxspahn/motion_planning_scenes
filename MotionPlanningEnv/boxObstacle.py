from dataclasses import dataclass, field
import numpy as np
import os
from copy import deepcopy
from MotionPlanningEnv.collisionObstacle import CollisionObstacle, CollisionObstacleConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv
from omegaconf import OmegaConf
from typing import List, Optional, Dict

class BoxObstacleMissmatchDimensionError(Exception):
    pass

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.
    This configuration class holds information about position
    and geometry of a box obstacle.
    Parameters:
    ------------
    position: list: Position of the obstacle
    lwh: list: length, width, height of the obstacle
    """
    position: List[float]
    lwh: List[float]


@dataclass
class BoxObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for box obstacle.
     
    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    mass: float : mass of the object, only used if movable set to true
    color : list : [r,g,b,a] where r,g,b and a are floats between 0 and 1
    id: integer : identify the box with a integer 
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """
    geometry: GeometryConfig
    movable: bool = False
    mass: float = 1
    color: List = field(default_factory=list) 
    id: int = -1
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None

class BoxObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['position', 'lwh']
        schema = OmegaConf.structured(BoxObstacleConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        self.checkGeometryCompleteness()
        self.checkDimensionality()
        
    def checkDimensionality(self):
        if self.dim() != len(self.position()):
            raise BoxObstacleMissmatchDimensionError(
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
            return [np.array(self._config.low.position), self._config.low.lwh]
        else:
            return [np.ones(self.dim()) * -1, 0]

    def limitHigh(self):
        if self._config.high:
            return [np.array(self._config.high.position), self._config.high.lwh]
        else:
            return [np.ones(self.dim()) * 1, 1]

    def position(self, **kwargs):
        return self._config.geometry.position

    def velocity(self, **kwargs):
        return np.zeros(self.dim())

    def acceleration(self, **kwargs):
        return np.zeros(self.dim())

    def lwh(self):
        return self._config.geometry.lwh

    def toDict(self):
        return OmegaConf.to_container(self._config)

    def movable(self):
        return self._config.movable
    
    def renderGym(self, viewer, rendering, **kwargs):
        raise NotImplementedError
        # TODO: the 2 dimensional version of box, rectangle
        # if self.dim() != 2:
        #     raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")
        # x = self.position()
        # tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        # joint = viewer.draw_circle(self.radius()) # <-- radius is not defined 
        # joint.add_attr(tf)

    def add2Bullet(self, pybullet):
        if self.dim() == 2:
            basePosition = self.position() + [0.0]
        elif self.dim() == 3:
            basePosition = self.position()
        else:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        collisionShape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=self.lwh())
        visualShapeId = self._config.id
        baseOrientation = [0, 0, 0, 1]

        pybullet.setAdditionalSearchPath(os.path.dirname(os.path.realpath(__file__)))

        visualShapeId = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName='box.obj',
            rgbaColor=self._config.color,  
            meshScale=self.lwh()
        )
        pybullet.createMultiBody(self._config.mass,
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)
