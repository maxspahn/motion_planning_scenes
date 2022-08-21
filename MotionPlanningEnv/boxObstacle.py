"""
Module doctring
"""
import os
from dataclasses import dataclass, field
from typing import List, Optional

from omegaconf import OmegaConf

from MotionPlanningSceneHelpers.motionPlanningComponent import (
    ComponentIncompleteError,
    DimensionNotSuitableForEnv,
)
from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)

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
    """
    Box obstacle class.
    """
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(BoxObstacleConfig)
        super().__init__(schema, **kwargs)
        self._geometry_keys = ["length", "width", "heigth"]

        self.check_completeness()
        self.check_geometry_completeness()

    def check_geometry_completeness(self):
        """
        Check if all mandatory keys are provided.
        """
        incomplete = False
        missing_keys = ""
        for key in self._geometry_keys:
            if key not in self.geometry():
                incomplete = True
                missing_keys += key + ", "
        if incomplete:
            raise ComponentIncompleteError(
                f"Missing keys in geometry: {missing_keys[:-2]}")

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

    def heigth(self):
        """
        Height of the obstacle.
        """
        return self._config.geometry.heigth

    def to_dict(self):
        """
        Convert to dictionary
        """
        return OmegaConf.to_container(self._config)

    def add2Bullet(self, pybullet):
        """
        Adds object to pybullet environment.
        """
        pybullet.setAdditionalSearchPath(
                os.path.dirname(os.path.realpath(__file__)))

        collision_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,
                halfExtents=[self.length(), self.width(), self.heigth()])

        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="box.obj",
            rgbaColor=self.color(),
            meshScale=[self.length(), self.width(), self.heigth()]
        )

        if self.dimension() == 3:
            base_position = self.position()
        else:
            raise DimensionNotSuitableForEnv(
                    "Pybullet only supports three dimensional obstacles")

        base_orientation = self.orientation()

        pybullet.createMultiBody(self._config.mass,
              collision_shape,
              visual_shape_id,
              base_position,
              base_orientation)
