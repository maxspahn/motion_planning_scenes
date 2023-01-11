from dataclasses import dataclass
from typing import Optional
from omegaconf import OmegaConf

from mpscenes.obstacles.dynamic_obstacle import DynamicObstacle, DynamicGeometryConfig
from mpscenes.obstacles.box_obstacle import BoxGeometryConfig, BoxObstacle, BoxObstacleConfig


@dataclass
class DynamicBoxGeometryConfig(BoxGeometryConfig, DynamicGeometryConfig):
    pass

@dataclass
class DynamicBoxObstacleConfig(BoxObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a dynamic spherical obstacle.

    Parameters:
    ------------

    geometry : DynamicBoxGeometryConfig : Geometry of the obstacle
    low: DynamicBoxGeometryConfig : Lower limit for randomization
    high: DynamicBoxGeometryConfig : Upper limit for randomization
    """

    geometry: DynamicBoxGeometryConfig
    low: Optional[DynamicBoxGeometryConfig] = None
    high: Optional[DynamicBoxGeometryConfig] = None


class DynamicBoxObstacle(DynamicObstacle, BoxObstacle):

    def __init__(self, **kwargs):
        schema = OmegaConf.structured(DynamicBoxObstacleConfig)
        kwargs['schema'] = schema
        super().__init__(**kwargs)
