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
class GeometryConfig:
    """
    Configuration dataclass for geometry.
    This configuration class holds information
    about the geometry of a cylinder obstacle.

    Parameters:
    ------------
    radius: float: radius of the obstacle
    height: float: Height of the obstacle
    """
    radius: float
    height: float

@dataclass
class CylinderObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for cylinder obstacle.

    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    orientation: list: [a,b,c,d] Quaternion orientation of the obstacle
    movable : bool : Flag indicating whether an obstacle is movable
    mass: float : Mass of the object, only used if movable set to true
    color : list : [r,g,b,a] rgba Color where r,g,b,a floats between 0 and 1
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

class CylinderObstacle(CollisionObstacle):
    """
    Cylinder obstacle class.
    """
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(CylinderObstacleConfig)
        super().__init__(schema, **kwargs)
        self._geometry_keys = ["radius", "height"]

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

    def radius(self):
        """
        Radius of the Obstacle.
        """
        return self._config.geometry.radius

    def height(self):
        """
        Height of the obstacle.
        """
        return self._config.geometry.height

    def to_dict(self):
        """
        Convert to dictionary.
        """
        return OmegaConf.to_container(self._config)

    def movable(self):
        """
        Indicates it the object is movable.
        """
        return self._config.movable

    def add_to_bullet(self, pybullet):
        """
        Adds object to pybullet environment.
        """
        pybullet.setAdditionalSearchPath(
                os.path.dirname(os.path.realpath(__file__)))

        collision_shape = pybullet.createCollisionShape(
                pybullet.GEOM_CYLINDER,
                radius=self.radius(),
                height=self.height())
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="cylinder.obj",
            rgbaColor=self.color(),
            meshScale=[self.radius(), self.radius(), self.height()]
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

