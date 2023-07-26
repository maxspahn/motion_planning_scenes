from typing import List, Optional
from dataclasses import dataclass, field
from abc import abstractmethod
import numpy as np

from mpscenes.common.component import MPComponent


def quaternion_to_homogeneous_matrix(
    translation: np.ndarray, quaternion: np.ndarray
) -> np.ndarray:
    w, x, y, z = quaternion

    # Normalize the quaternion
    norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    # Calculate transformation matrix elements
    xx = 1 - 2 * y**2 - 2 * z**2
    xy = 2 * x * y - 2 * w * z
    xz = 2 * x * z + 2 * w * y
    yx = 2 * x * y + 2 * w * z
    yy = 1 - 2 * x**2 - 2 * z**2
    yz = 2 * y * z - 2 * w * x
    zx = 2 * x * z - 2 * w * y
    zy = 2 * y * z + 2 * w * x
    zz = 1 - 2 * x**2 - 2 * y**2

    # Construct the transformation matrix
    transformation_matrix = np.array(
        [
            [xx, xy, xz, translation[0]],
            [yx, yy, yz, translation[1]],
            [zx, zy, zz, translation[2]],
            [0, 0, 0, 1],
        ]
    )

    return transformation_matrix


@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry.

    This configuration class holds information about position and orientation
    of an obstacle. This class is further specified for the other obstacles.

    Parameters:
    ------------

    position: list: Position of the obstacle
    orientation: list: Orientation of the obstacle
    color: list: RGB-A color code
    """

    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])


@dataclass
class CollisionObstacleConfig:
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about the dimension and the type
    of collision obstacle.

    Parameters:
    ------------

    type : str : Type of the obstacle
    geometry : GeometryConfig : Geometry of the obstacle
    movable : bool : Flag indicating whether an obstacle can be pushed around
    low : GeometryConfig : Lower limit for randomization
    high : GeometryConfig : Upper limit for randomization
    rgba: list: Color in rgba encoding
    """

    type: str
    geometry: GeometryConfig
    movable: bool = False
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None
    rgba: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])


class CollisionObstacle(MPComponent):
    def type(self) -> str:
        return self._config.type

    def geometry(self) -> GeometryConfig:
        return self._config.geometry

    def evaluate(self, **kwargs) -> list:
        return [
            self.position(**kwargs),
            self.velocity(**kwargs),
            self.acceleration(**kwargs),
        ]

    def dimension(self) -> int:
        return len(self.geometry().position)

    def orientation(self, **kwarg) -> np.ndarray:
        return np.array(self.geometry().orientation)

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self.geometry().position)

    def rgba(self) -> np.ndarray:
        return np.array(self._config.rgba)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def movable(self) -> bool:
        return self._config.movable

    def position_into_obstacle_frame(self, positions: np.ndarray) -> np.ndarray:
        transformation_matrix = quaternion_to_homogeneous_matrix(
            self.position(), self.orientation()
        )
        if len(positions.shape) > 1:
            nb_points = positions.shape[0]
            positions_homo = np.transpose(np.append(positions, np.ones((nb_points, 1)), axis=1))
            return np.dot(np.linalg.inv(transformation_matrix), positions_homo)[0:3,:]
        else:
            positions_homo = np.transpose(np.append(positions, 1))
            return np.dot(np.linalg.inv(transformation_matrix), positions_homo)[0:3]


    @abstractmethod
    def distance(self, position: np.ndarray) -> float:
        pass

    @abstractmethod
    def size(self) -> np.ndarray:
        pass
