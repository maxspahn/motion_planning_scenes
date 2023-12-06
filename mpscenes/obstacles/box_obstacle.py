from dataclasses import dataclass
from typing import Optional, List

from omegaconf import OmegaConf
import numpy as np
from mpscenes.obstacles.collision_obstacle import CollisionObstacle, CollisionObstacleConfig, GeometryConfig



@dataclass
class BoxGeometryConfig(GeometryConfig):
    """Configuration dataclass for geometry.

    This configuration class holds information about position
    and size of the box obstacle.

    Attributes:
    ------------

    length: float
        Length of the box
    width: float
        Width of the box
    height: float
        Height of the box
    """

    length: float = 1.0
    width: float = 1.0
    height: float = 1.0


@dataclass
class BoxObstacleConfig(CollisionObstacleConfig):
    """Configuration dataclass for box obstacle.

    This configuration class holds information about the position, size
    and randomization of a rectengular obstacle.

    Attributes:
    ------------

    geometry : BoxGeometryConfig : Geometry of the box
    low : BoxGeometryConfig : Lower limit for randomization
    high : BoxGeometryConfig : Upper limit for randomization
    """

    geometry: BoxGeometryConfig
    low: Optional[BoxGeometryConfig] = None
    high: Optional[BoxGeometryConfig] = None


class BoxObstacle(CollisionObstacle):
    _config: BoxObstacleConfig

    def __init__(self, **kwargs):
        if 'schema' not in kwargs:
            schema = OmegaConf.structured(BoxObstacleConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        self.check_completeness()

    def size(self) -> List[float]:
        """Get size of box obstacle, length, width, height."""
        return [
            self.length(),
            self.width(),
            self.height(),
        ]

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

    def length(self) -> float:
        """
        Get the length of the obstacle.
        """
        return self._config.geometry.length

    def width(self) -> float:
        """
        Get the width of the obstacle.
        """
        return self._config.geometry.width

    def height(self) -> float:
        """
        Get the height of the obstacle.
        """
        return self._config.geometry.height

    def shuffle(self) -> None:
        """
        Randomize the obstacle by shuffling the position, length, width and
        height.
        """
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

    def distance(self, position: np.ndarray, **kwargs) -> float:
        """
        Get distance between point and the box obstacleself.

        The point is transformed into the main axes of the obstacle. Then, the
        halfplanes are used to compute the distance.
        """
        pos = self.position_into_obstacle_frame(position, **kwargs)
        q = np.transpose(np.subtract(np.transpose(np.absolute(pos)),
                                     np.array(self.size())/2.0)) 
        return np.linalg.norm(np.maximum(q, 0), axis=0) + np.minimum(np.maximum(q[0], np.maximum(q[1], q[2])), 0.0)
