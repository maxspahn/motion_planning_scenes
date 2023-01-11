from dataclasses import dataclass
from typing import Optional
from omegaconf import OmegaConf

from mpscenes.obstacles.dynamic_obstacle import DynamicObstacle, DynamicGeometryConfig
from mpscenes.obstacles.urdf_obstacle import UrdfGeometryConfig, UrdfObstacle, UrdfObstacleConfig


@dataclass
class DynamicUrdfGeometryConfig(UrdfGeometryConfig, DynamicGeometryConfig):
    pass

@dataclass
class DynamicUrdfObstacleConfig(UrdfObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a dynamic spherical obstacle.

    Parameters:
    ------------

    geometry : DynamicUrdfGeometryConfig : Geometry of the obstacle
    low: DynamicUrdfGeometryConfig : Lower limit for randomization
    high: DynamicUrdfGeometryConfig : Upper limit for randomization
    """

    geometry: DynamicUrdfGeometryConfig
    low: Optional[DynamicUrdfGeometryConfig] = None
    high: Optional[DynamicUrdfGeometryConfig] = None


class DynamicUrdfObstacle(DynamicObstacle, UrdfObstacle):

    def __init__(self, **kwargs):
        schema = OmegaConf.structured(DynamicUrdfObstacleConfig)
        kwargs['schema'] = schema
        super().__init__(**kwargs)
