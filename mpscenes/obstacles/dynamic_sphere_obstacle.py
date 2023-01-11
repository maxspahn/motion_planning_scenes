from dataclasses import dataclass
from omegaconf import OmegaConf
from typing import Optional

from mpscenes.obstacles.dynamic_obstacle import DynamicObstacle, DynamicGeometryConfig
from mpscenes.obstacles.sphere_obstacle import SphereGeometryConfig, SphereObstacle, SphereObstacleConfig


@dataclass
class DynamicSphereGeometryConfig(SphereGeometryConfig, DynamicGeometryConfig):
    pass

@dataclass
class DynamicSphereObstacleConfig(SphereObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a dynamic spherical obstacle.

    Parameters:
    ------------

    geometry : DynamicSphereGeometryConfig : Geometry of the obstacle
    low: DynamicSphereGeometryConfig : Lower limit for randomization
    high: DynamicSphereGeometryConfig : Upper limit for randomization
    """

    geometry: DynamicSphereGeometryConfig
    low: Optional[DynamicSphereGeometryConfig] = None
    high: Optional[DynamicSphereGeometryConfig] = None


class DynamicSphereObstacle(DynamicObstacle, SphereObstacle):

    def __init__(self, **kwargs):
        schema = OmegaConf.structured(DynamicSphereObstacleConfig)
        kwargs['schema'] = schema
        super().__init__(**kwargs)
