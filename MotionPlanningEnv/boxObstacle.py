import os
from dataclasses import dataclass, field
from typing import List, Optional

from omegaconf import OmegaConf

from MotionPlanningSceneHelpers.motionPlanningComponent import (
    DimensionNotSuitableForEnv,
)
from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)

@dataclass
class GeometryConfig():
    """
    This configuration class holds information about
    the geometry of a box obstacle.
    Parameters:
    ------------
    length: float: Lenght of the obstacle
    width: float: Width of the obstacle
    height: float: Height of the obstacle
    """
    length: float
    width: float
    height: float

@dataclass
class BoxObstacleConfig(CollisionObstacleConfig):
    """
    Configuration dataclass for box obstacle.
    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    orientation: List: [a,b,c,d] Quaternion orientation of the obstacle
    movable : bool : Flag indicating whether an obstacle is movable
    mass: float : mass of the object, only used if movable set to true
    color : List : [r,g,b,a] where r,g,b and a are floats between 0 and 1
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
    """
    Box obstacle class.
    """
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(BoxObstacleConfig)
        super().__init__(schema, **kwargs)
        self.add_required_keys(["length", "width", "height"])

        self.check_completeness()

    def length(self):
        """
        Length of the obstacle.
        """
        return self._config.geometry.length

    def width(self):
        """
        Width of the obstacle.
        """
        return self._config.geometry.width

    def height(self):
        """
        Height of the obstacle.
        """
        return self._config.geometry.height

    def add_to_bullet(self, pybullet):
        """
        Adds object to pybullet environment.
        """
        pybullet.setAdditionalSearchPath(
                os.path.dirname(os.path.realpath(__file__)))

        collision_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,
                halfExtents=[self.length(), self.width(), self.height()])

        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="box.obj",
            rgbaColor=self.color(),
            meshScale=[self.length(), self.width(), self.height()]
        )

        if self.dimension() == 3:
            base_position = self.position()
        else:
            raise DimensionNotSuitableForEnv(
                    "Pybullet only supports three dimensional obstacles")

        base_orientation = self.orientation()

        mass = -1
        if self.movable():
            mass = self.mass()

        pybullet.createMultiBody(mass,
              collision_shape,
              visual_shape_id,
              base_position,
              base_orientation)
