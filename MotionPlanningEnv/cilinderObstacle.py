from dataclasses import dataclass, field
import os
from MotionPlanningEnv.collisionObstacle import CollisionObstacle, CollisionObstacleConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv
from omegaconf import OmegaConf
from typing import Optional

class CilinderObstacleMissmatchDimensionError(Exception):
    pass

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.
    This configuration class holds information about position
    and geometry of a cilinder obstacle.

    Parameters:
    ------------
    position: list: [x,y,z] Position of the obstacle
    radius: float: radius of the obstacle
    heigth: float: Heigth of the obstacle
    """
    position: list[float]
    radius: float
    heigth: float

@dataclass
class CilinderObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for cilinder obstacle.
     
    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    orientation: list: [a,b,c,d] Quaternion orientation of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    mass: float : Mass of the object, only used if movable set to true
    color : list : [r,g,b,a] rgba Color where r,g,b and a are floats between 0 and 1
    id: integer : Identify the box with a integer 
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """
    geometry: GeometryConfig
    orientation: list[float] = field(default_factory=list) 
    movable: bool = False
    mass: float = 1
    color: list[float] = field(default_factory=list) 
    id: int = -1
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None

class CilinderObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['position', 'radius', 'heigth']
        schema = OmegaConf.structured(CilinderObstacleConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        self.checkGeometryCompleteness()
        self.checkDimensionality()
        
    def checkDimensionality(self):
        if self.dim() != len(self.position()):
            raise CilinderObstacleMissmatchDimensionError(
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
        
    def position(self):
        return self._config.geometry.position

    def velocity(self):
        raise NotImplementedError

    def acceleration(self):
        raise NotImplementedError

    def radius(self):
        return self._config.geometry.radius

    def height(self):
        return self._config.geometry.heigth

    def toDict(self):
        return OmegaConf.to_container(self._config)

    def movable(self):
        return self._config.movable
   
    def add2Bullet(self, pybullet):
        if self.dim() == 3:
            basePosition = self.position()
        else:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        
        collisionShape = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=self.radius(), height=self.height())
        visualShapeId = self.id()
        baseOrientation = self.orientation()

        pybullet.setAdditionalSearchPath(os.path.dirname(os.path.realpath(__file__)))

        visualShapeId = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName='cilinder.obj',
            rgbaColor=self.color(),  
            meshScale=[self.radius(), self.radius(), self.height()]
        )
        pybullet.createMultiBody(self.mass(),
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)

