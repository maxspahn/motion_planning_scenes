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
class GeometryConfig:
    """
    This configuration dataclass holds information about
    the geometry of the cylinder obstacle.
    Parameters:
    ------------
    radius: float: Radius of the obstacle
    height: float: Height of the obstacle
    """
    radius: float
    height: float

@dataclass
class CylinderObstacleConfig(CollisionObstacleConfig):
    """
    Configuration dataclass for cylinder obstacle.

    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    orientation: List: [a,b,c,d] Quaternion orientation of the obstacle
    movable : bool : Flag indicating whether an obstacle is movable
    mass: float : Mass of the object, only used if movable set to true
    color : List : [r,g,b,a] rgba Color where r,g,b,a floats between 0 and 1
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

        mass = -1
        if self.movable():
            mass = self.mass()

        pybullet.createMultiBody(mass,
              collision_shape,
              visual_shape_id,
              base_position,
              base_orientation)
