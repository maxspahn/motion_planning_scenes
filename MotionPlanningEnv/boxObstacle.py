from dataclasses import dataclass, field
import os

from omegaconf import OmegaConf
from typing import List, Optional

from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    ComponentIncompleteError,
    DimensionNotSuitableForEnv,
)

class BoxObstacleMissmatchDimensionError(Exception):
    pass

@dataclass
class GeometryConfig():
    """Configuration dataclass for geometry.
    This configuration class holds information about geometry of a box obstacle.

    Parameters:
    ------------
    length: float: Lenght of the obstacle
    width: float: Width of the obstacle
    heigth: float: Heigth of the obstacle
    """
    length: float
    width: float
    heigth: float

@dataclass
class BoxObstacleConfig(CollisionObstacleConfig):
    """
    Configuration dataclass for box obstacle.
     
    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    orientation: list: Orientation of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    mass: float : mass of the object, only used if movable set to true
    color : list : [r,g,b,a] where r,g,b and a are floats between 0 and 1
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """
    geometry: GeometryConfig
    orientation: List[float] = field(default_factory=list)
    movable: bool = False
    mass: float = 1
    color: List[float] = field(default_factory=list) 
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None

class BoxObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(BoxObstacleConfig)
        super().__init__(schema, **kwargs)
        self._geometry_keys = ['length', 'width', 'heigth']

        self.check_completeness()
        self.checkGeometryCompleteness()
        self.checkDimensionality()

    def checkDimensionality(self):
        if self.dimension() != len(self.position()):
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

    def length(self):
        return self._config.geometry.length

    def width(self):
        return self._config.geometry.width

    def heigth(self):
        return self._config.geometry.heigth

    def toDict(self):
        return OmegaConf.to_container(self._config)

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
        pybullet.setAdditionalSearchPath(os.path.dirname(os.path.realpath(__file__)))

        collisionShape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[self.length(), self.width(), self.heigth()])

        visualShapeId = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName='box.obj',
            rgbaColor=self.color(),  
            meshScale=[self.length(), self.width(), self.heigth()]
        )

        if self.dimension() == 3:
            basePosition = self.position()
        else:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        
        baseOrientation = self.orientation()
         
        pybullet.createMultiBody(self._config.mass,
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)
