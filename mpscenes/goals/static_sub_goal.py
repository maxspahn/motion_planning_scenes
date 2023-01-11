from typing import List, Optional, Any
from dataclasses import dataclass
import numpy as np
from pyquaternion import Quaternion
from omegaconf import OmegaConf

from mpscenes.goals.sub_goal import SubGoal, SubGoalConfig
from mpscenes.common.errors import DimensionNotSuitableForEnv


@dataclass
class StaticSubGoalConfig(SubGoalConfig):
    """Configuration dataclass for static sub goal.

    This configuration class holds information about the
    the weight, accuracy required, type and position in the
    kinematic chain.

    Parameters:
    ------------

    parent_link: str
        Name of the link that specifies the frame in which the goal is defined
    child_link: str
        Name of the link that should match the desired position
    desired_position : list
        Goal state of the concerned link
    angle: list
        Additional rotation from the parent_link frame given by a quaternion
    low: list
        Lower limit for randomization
    high: list
        Upper limit for randomization

    """

    parent_link: Any
    child_link: Any
    desired_position: List[float]
    angle: Optional[Any] = None
    low: Optional[List[float]] = None
    high: Optional[List[float]] = None


class StaticSubGoal(SubGoal):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(StaticSubGoalConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        self.check_completeness()
        self.check_dimensionality()

    def parent_link(self):
        return self._config.parent_link

    def child_link(self):
        return self._config.child_link

    def limit_low(self):
        if self._config.low:
            return np.array(self._config.low)
        else:
            return np.ones(self.dimension()) * -1

    def limit_high(self):
        if self._config.high:
            return np.array(self._config.high)
        else:
            return np.ones(self.dimension()) * 1

    def position(self, **kwargs) -> np.ndarray:
        return np.array(self._config.desired_position)

    def velocity(self, **kwargs) -> np.ndarray:
        return np.zeros(self.dimension())

    def acceleration(self, **kwargs):
        return np.zeros(self.dimension())

    def shuffle(self):
        random_pos = np.random.uniform(
            self.limit_low(), self.limit_high(), self.dimension()
        )
        self._config.desired_position = random_pos.tolist()

    def angle(self):
        if isinstance(self._config.angle, float):
            return self._config.angle
        if self._config.angle:
            return list(self._config.angle)

    def render_gym(self, viewer, rendering, **kwargs):
        """Rendering the static sub goal into a planar gym environment.

        The static sub goal is rendered into the viewer of the open-ai gym by
        adding either a point or a line in the rendering window.

        Parameters:
        ------------
        viewer: Viewing window of open-ai gym
        rendering: Rendering toolbox in open-ai gym
        """
        coordinate_system_1 = viewer.draw_line(
            [-3, 0], [3, 0], color=[0.0, 0.0, 0.0]
        )
        coordinate_system_2 = viewer.draw_line(
            [0, -3], [0, 3], color=[0.0, 0.0, 0.0]
        )
        angle = self.angle()
        if angle:
            tf2 = rendering.Transform(rotation=self.angle())
        else:
            tf2 = rendering.Transform()
        coordinate_system_1.add_attr(tf2)
        coordinate_system_2.add_attr(tf2)
        if self.dimension() == 1:
            start_point = [-10, -10.0]
            end_point = [10.0, 10.0]
            start_point[self.indices()[0]] = 0.0
            end_point[self.indices()[0]] = 0.0
            goal = viewer.draw_line(
                start_point, end_point, color=[0.0, 1.0, 0.0]
            )
            translation = [0.0, 0.0]
            translation[self.indices()[0]] = self.position()[0]
            tf = rendering.Transform(translation=translation)
            goal.add_attr(tf)
            goal.add_attr(tf2)
        elif self.dimension() == 2:
            x = self.position()
            goal = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
            tf = rendering.Transform(translation=(x[0], x[1]))
            goal.add_attr(tf)
            goal.add_attr(tf2)
        else:
            raise DimensionNotSuitableForEnv(
                "PlanarGym only supports two dimensional obstacles"
            )

    def add_to_bullet(self, pybullet, position=[0.0, 0.0, 0.0]) -> int:
        """Adds the static sub goal to pybullet.

        The static sub goal is added to the pybullet as point or a coordinate
        system if the orientation is specified.

        Parameters:
        ------------
        pybullet: pybulle-instance
        position: list:
            goal position is default the origin for orientation-only goals.
        """
        if self.dimension() == 2:
            base_position = self.position().tolist() + [0.0]
        elif self.dimension() == 3:
            base_position = self.position().tolist()
        else:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports three dimensional obstacles"
            )
        rgba_color = [0.0, 1.0, 0.0, 0.3]
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_SPHERE, rgbaColor=rgba_color, radius=self.epsilon()
        )
        collision_shape = -1
        base_orientation = [0, 0, 0, 1]
        mass = 0

        assert isinstance(base_position, list)
        assert isinstance(base_orientation, list)
        multi_body_id = pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            base_position,
            base_orientation,
        )

        if self.angle():
            for i in range(3):
                self.add_axis_component(pybullet, i, position)
        return multi_body_id

    def add_axis_component(self, pybullet, i, goal_position):
        """Adds the coordinate system of the static sub goal to pybullet.

        If the user has specified an angle for the subgoal, the coordinate
        system is added to pybullet as three differently colored cylinders.

        Parameters:
        ------------
        pybullet: yybullet-instance
        i: int
            Axis specifier for either x(i=0), y(i=1), z(i=2)
        goal_position: list
            goal position to align with a potential position goal
        """
        rgba_color = [0.0, 0.0, 0.0, 0.3]
        angles = self.angle()
        if not angles:
            angles = Quaternion([1, 0.0, 0, 0])
        else:
            angles = Quaternion(angles).inverse
        orientation = Quaternion([1, 0, 0, 0])
        orientation[i + 1] = 1.0
        orientation = orientation.normalised
        position = np.array(
            [goal_position[0], goal_position[1], goal_position[2]]
        )
        orientation = angles * orientation
        orientation_array = [
            orientation[1],
            orientation[2],
            orientation[3],
            orientation[0],
        ]
        offset_array = [
            np.array([0.00, 0.00, -0.05]),
            np.array([0.0, 0.0, 0.05]),
            np.array([0.0, 0.0, 0.05]),
        ]

        position += orientation.rotate(offset_array[i])
        index_map = [1, 0, 2]
        rgba_color[index_map[i]] = 1.0
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_CYLINDER,
            rgbaColor=rgba_color,
            radius=0.01,
            length=0.1,
        )
        collision_shape = -1
        mass = 0
        pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            position,
            orientation_array,
        )
