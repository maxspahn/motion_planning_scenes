from dataclasses import dataclass
from typing import Optional
from omegaconf import OmegaConf

from mpscenes.obstacles.dynamic_obstacle import DynamicObstacle, DynamicGeometryConfig
from mpscenes.obstacles.cylinder_obstacle import CylinderGeometryConfig, CylinderObstacle, CylinderObstacleConfig


@dataclass
class DynamicCylinderGeometryConfig(CylinderGeometryConfig, DynamicGeometryConfig):
    pass

@dataclass
class DynamicCylinderObstacleConfig(CylinderObstacleConfig):
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the position, size
    and randomization of a dynamic spherical obstacle.

    Parameters:
    ------------

    geometry : DynamicCylinderGeometryConfig : Geometry of the obstacle
    low: DynamicCylinderGeometryConfig : Lower limit for randomization
    high: DynamicCylinderGeometryConfig : Upper limit for randomization
    """

    geometry: DynamicCylinderGeometryConfig
    low: Optional[DynamicCylinderGeometryConfig] = None
    high: Optional[DynamicCylinderGeometryConfig] = None


class DynamicCylinderObstacle(DynamicObstacle, CylinderObstacle):

    def __init__(self, **kwargs):
        schema = OmegaConf.structured(DynamicCylinderObstacleConfig)
        kwargs['schema'] = schema
        super().__init__(**kwargs)
