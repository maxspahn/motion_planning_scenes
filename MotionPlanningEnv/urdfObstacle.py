from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    DimensionNotSuitableForEnv,
)

from dataclasses import dataclass
from omegaconf import OmegaConf

@dataclass
class UrdfObstacleConfig(CollisionObstacleConfig):
    """
    This configuration class holds information about the urdf obstacle.

    Parameters:
    ------------
    urdf : str : Filename of the urdf

    """
    urdf: str

class UrdfObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(UrdfObstacleConfig)
        super().__init__(schema, **kwargs)
        self.check_completeness()

    def urdf(self):
        return self._config.urdf

    def add_to_bullet(self, pybullet):
        if self.dimension() != 3:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports two dimensional obstacles"
            )
        pybullet.loadURDF(fileName=self.urdf(), basePosition=self.position())
