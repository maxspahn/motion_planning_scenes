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

    Attributes:
    ------------

    position: List[float], default=[0.0, 0.0, 0.0]
        Position of the obstacle
    orientation: List[float], default=[1.0, 0.0, 0.0, 0.0]
        Orientation of the obstacle as quaternion wxyz
    """

    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])


@dataclass
class CollisionObstacleConfig:
    """Configuration dataclass for collision obstacle.

    This configuration class holds information about the dimension and the type
    of collision obstacle.

    Parameters:
    ------------

    type : str
        Type of the obstacle
    geometry : GeometryConfig
        Geometry of the obstacle
    movable : bool
        Flag indicating whether an obstacle can be pushed around
    low : GeometryConfig 
        Lower limit for randomization
    high : GeometryConfig 
        Upper limit for randomization
    rgba: List[float]
        Color in rgba encoding
    """

    type: str
    geometry: GeometryConfig
    movable: bool = False
    low: Optional[GeometryConfig] = None
    high: Optional[GeometryConfig] = None
    rgba: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])

class CollisionObstacle(MPComponent):
    """
    Represents a collision obstacle.

    Base class for collision obstacles.

    Attributes
    _____________
    _config : CollisionObstacleConfig
        Configuration of collision obstacle

    """

    _config: CollisionObstacleConfig

    def type(self) -> str:
        """
        Get the type of the collision obstacle.
        """
        return self._config.type

    def geometry(self) -> GeometryConfig:
        """
        Get the geometry configuration of the collision obstacle.
        """
        return self._config.geometry

    def evaluate(self, **kwargs) -> List[np.ndarray]:
        """
        Evaluate the obstacle's position, velocity, and acceleration.
        """
        return [
            self.position(**kwargs),
            self.velocity(**kwargs),
            self.acceleration(**kwargs),
        ]

    def dimension(self) -> int:
        """
        Get the dimension of the obstacle.
        """
        return len(self.geometry().position)

    def orientation(self, **kwarg) -> np.ndarray:
        """
        Get the orientation of the obstacle as quaternion with convention wxyz.
        """
        return np.array(self.geometry().orientation)

    def position(self, **kwargs) -> np.ndarray:
        """
        Get the position of the obstacle, xyz.
        """
        return np.array(self.geometry().position)

    def rgba(self) -> np.ndarray:
        """
        Get the color (RGBA) configuration of the obstacle.
        """
        return np.array(self._config.rgba)

    def velocity(self, **kwargs) -> np.ndarray:
        """
        Get the linear velocity of the obstacle. It returns zero, unless it is
        a moving obstacle. Does not capture the velocity of movable obstacles.
        That must be handled by the corresponding physics engine.
        """
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs) -> np.ndarray:
        """
        Get the linear acceleration of the obstacle. It returns zero, unless it is
        a moving obstacle. Does not capture the acceleration of movable obstacles.
        That must be handled by the corresponding physics engine.
        """
        return np.zeros(self.dimension())

    def movable(self) -> bool:
        """
        Check if the obstacle is movable, True if obstacle can be interacted with.
        """
        return self._config.movable

    def position_into_obstacle_frame(self, positions: np.ndarray, **kwargs) -> np.ndarray:
        """
        Transform positions into the obstacle's frame. That is needed to compute
        the distance between points and the obstacle.
        """
        if 't' in kwargs:
            t = kwargs.get('t')
        else:
            t = 0
        transformation_matrix = quaternion_to_homogeneous_matrix(
            self.position(t=t), self.orientation()
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
        """
        Abstract method to calculate the distance of a point to the obstacle.
        """

    @abstractmethod
    def size(self) -> np.ndarray:
        """
        Abstract method to retrieve the size of the obstacle.
        """
