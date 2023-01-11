from dataclasses import dataclass
from typing import List, Optional, Any
import numpy as np
from omegaconf import OmegaConf
from pyquaternion import Quaternion
from mpscenes.goals.sub_goal import SubGoal, SubGoalConfig
from mpscenes.common.errors import DimensionNotSuitableForEnv, TrajectoryNotSupported
from mpscenes.common.analytic_trajectory import AnalyticTrajectory
from mpscenes.common.spline_trajectory import SplineTrajectory


@dataclass
class DynamicSubGoalConfig(SubGoalConfig):
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
    trajectory: Any
        Trajectory of the goal, either defined by spline or analytic equation.
    angle list
        Additional rotation from the parent_link frame given by a quaternion
    low : list
        Lower limit for randomization
    high : list
        Upper limit for randomization

    """

    parent_link: str
    child_link: str
    trajectory: Any
    angle: Optional[Any] = None
    low: Optional[List[float]] = None
    high: Optional[List[float]] = None


class DynamicSubGoal(SubGoal):
    def __init__(self, **kwargs):
        if not 'schema' in kwargs:
            schema = OmegaConf.structured(DynamicSubGoalConfig)
            kwargs['schema'] = schema
        super().__init__(**kwargs)
        if "controlPoints" in self._config.trajectory:
            self._trajectory_type = "spline"
        elif isinstance(self._config.trajectory[0], str):
            self._trajectory_type = "analytic"
        else:
            raise TrajectoryNotSupported(
                f"Trajectory definition not supported, {self._config.trajectory}"
            )
        self.check_completeness()
        if self.trajectory_type() == 'spline':
            self._traj = SplineTrajectory(
                self.dimension(), traj=self._config.trajectory
            )
        elif self.trajectory_type() == "analytic":
            self._traj = AnalyticTrajectory(
                self.dimension(), traj=self._config.trajectory
            )
        self._traj.concretize()
        self.check_dimensionality()

    def trajectory_type(self) -> str:
        return self._trajectory_type

    def parent_link(self):
        return self._config.parent_link

    def child_link(self):
        return self._config.child_link

    def traj(self):
        return self._traj

    def dict(self):
        return OmegaConf.to_container(self._config)

    def position(self, **kwargs) -> np.ndarray:
        return self.evaluate_trajectory(**kwargs)[0]

    def velocity(self, **kwargs) -> np.ndarray:
        return self.evaluate_trajectory(**kwargs)[1]

    def acceleration(self, **kwargs) -> np.ndarray:
        return self.evaluate_trajectory(**kwargs)[2]

    def evaluate_trajectory(self, **kwargs):
        t = kwargs.get("t") if "t" in kwargs else 0.0
        return self._traj.evaluate(t=t)

    def shuffle(self):
        self._traj.shuffle()

    def angle(self):
        return self._config.angle

    def render_gym(self, viewer, rendering, **kwargs):
        coordinate_system_1 = viewer.draw_line(
            [-3, 0], [3, 0], color=[0.0, 0.0, 0.0]
        )
        coordinate_system_2 = viewer.draw_line(
            [0, -3], [0, 3], color=[0.0, 0.0, 0.0]
        )
        angle = self.angle()
        if not angle:
            angle = 0.0
        tf2 = rendering.Transform(rotation=angle)
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
            translation[self.indices()[0]] = self.position(t=kwargs.get("t"))[0]
            tf = rendering.Transform(translation=translation)
            goal.add_attr(tf)
            goal.add_attr(tf2)
        elif self.dimension() == 2:
            x = self.position(t=kwargs.get("t"))
            goal = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
            tf = rendering.Transform(translation=(x[0], x[1]))
            goal.add_attr(tf)
            goal.add_attr(tf2)
        else:
            raise DimensionNotSuitableForEnv(
                "PlanarGym only supports two dimensional obstacles"
            )

    def update_bullet_position(self, pybullet, **kwargs):
        if "t" not in kwargs:
            t = 0.0
        else:
            t = kwargs.get("t")
        pos = self.position(t=t).tolist()
        if self.dimension() == 2 and self.indices() == [0, 1]:
            pos += [0.0]
        ori = [0, 0, 0, 1]
        pybullet.resetBasePositionAndOrientation(self._bullet_id, pos, ori)

    def add_to_bullet(self, pybullet, position=[0.0, 0.0, 0.0]) -> int:
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
        self._bullet_id = pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            base_position,
            base_orientation,
        )

        if self.angle():
            for i in range(3):
                self.add_axis_component(pybullet, i, position)
        return self._bullet_id


    def add_axis_component(self, pybullet, i, goal_position):
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
        assert isinstance(position, list)
        assert isinstance(orientation_array, list)
        return pybullet.createMultiBody(
            mass,
            collision_shape,
            visual_shape_id,
            position,
            orientation_array,
        )
