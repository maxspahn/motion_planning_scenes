from dataclasses import dataclass
import os
import numpy as np
from MotionPlanningEnv.collisionObstacle import (
    CollisionObstacle,
    CollisionObstacleConfig,
)
from MotionPlanningSceneHelpers.motionPlanningComponent import (
    DimensionNotSuitableForEnv,
)

from omegaconf import OmegaConf
from typing import List, Optional


class BoxObstacleMissmatchDimensionError(Exception):
    pass


@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and size of the box obstacle.

    Parameters:
    ------------

    position: list: Position of the obstacle
    length: float: Length of the box
    width: float: Width of the box
    height: float: Height of the box
    """

    position: List[float]
    length: float
    width: float
    height: float = 1.0


@dataclass
class BoxObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for box obstacle.

    This configuration class holds information about the position, size
    and randomization of a rectengular obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    """

    geometry: GeometryConfig
    movable: bool = False
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None


class BoxObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(BoxObstacleConfig)
        super().__init__(schema, **kwargs)
        self.check_completeness()

    def dimension(self):
        return len(self._config.geometry.position)

    def limit_low(self):
        if self._config.low:
            return [
                np.array(self._config.low.position),
                self._config.low.length,
                self._config.low.width,
                self._config.low.height,
            ]
        else:
            return [np.ones(self.dimension()) * -1, 0, 0, 0]

    def limit_high(self):
        if self._config.high:
            return [
                np.array(self._config.high.position),
                self._config.high.length,
                self._config.high.width,
                self._config.high.height,
            ]
        else:
            return [np.ones(self.dimension()) * 1, 1, 1, 1]

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self._config.geometry.position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def length(self):
        return self._config.geometry.length

    def width(self):
        return self._config.geometry.width

    def height(self):
        return self._config.geometry.height

    def shuffle(self):
        random_pos = np.random.uniform(
            self.limit_low()[0], self.limit_high()[0], self.dimension()
        )
        random_length = np.random.uniform(
            self.limit_low()[1], self.limit_high()[1], 1
        )
        random_width = np.random.uniform(
            self.limit_low()[2], self.limit_high()[2], 1
        )
        random_height = np.random.uniform(
            self.limit_low()[3], self.limit_high()[3], 1
        )
        self._config.geometry.position = random_pos.tolist()
        self._config.geometry.length = float(random_length)
        self._config.geometry.width = float(random_width)
        self._config.geometry.height = float(random_height)

    def movable(self):
        return self._config.movable

    def csv(self, file_name, samples=100):
        pass

    def render_gym(self, viewer, rendering, **kwargs):
        if self.dimension() != 2:
            raise DimensionNotSuitableForEnv(
                "PlanarGym only supports two dimensional obstacles"
            )
        x = self.position()
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        left, right, top, bottom = (
            -0.5 * self.length(),
            0.5 * self.length(),
            0.5 * self.width(),
            -0.5 * self.width(),
        )
        joint = viewer.draw_polygon([(left, bottom), (left, top), (right, top), (right, bottom)])

        joint.add_attr(tf)

    def add_to_bullet(self, pybullet) -> int:
        if self.dimension() == 2:
            base_position = self.position().tolist() + [0.0]
        elif self.dimension() == 3:
            base_position = self.position().tolist()
        else:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports three dimensional obstacles"
            )
        collision_shape = pybullet.createCollisionShape(
            pybullet.GEOM_BOX, halfExtents=[self.length(), self.width(), self.height()]
        )
        visual_shape_id = -1
        base_orientation = [0, 0, 0, 1]
        mass = int(self.movable())
        pybullet.setAdditionalSearchPath(
            os.path.dirname(os.path.realpath(__file__))
        )
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="box_smooth.obj",
            rgbaColor=[1.0, 0.0, 0.0, 1.0],
            specularColor=[1.0, 0.5, 0.5],
            meshScale=[self.length(), self.width(), self.height()],
        )
        assert isinstance(base_position, list)
        assert isinstance(base_orientation, list)
        return pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            base_position,
            base_orientation,
        )
