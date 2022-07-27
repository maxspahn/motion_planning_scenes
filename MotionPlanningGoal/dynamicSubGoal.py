from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from omegaconf import OmegaConf
from MotionPlanningGoal.subGoal import SubGoal, SubGoalConfig
from MotionPlanningSceneHelpers.motionPlanningComponent import DimensionNotSuitableForEnv
from MotionPlanningSceneHelpers.analyticTrajectory import AnalyticTrajectory
from MotionPlanningSceneHelpers.splineTrajectory import SplineTrajectory

@dataclass
class DynamicSubGoalConfig(SubGoalConfig):
    """Configuration dataclass for static sub goal.

    This configuration class holds information about the 
    the weight, accuracy required, type and position in the 
    kinematic chain.

    Parameters:
    ------------

    parent_link: str : Name of the link that specifies the frame in which the goal is defined
    child_link: str : Name of the link that should match the desired position
    trajectory: Any: Trajectory of the goal, either defined by a spline or an analytic equation.
    angle list : Additional rotation from the parent_link frame given by a quaternion
    low : list : Lower limit for randomization
    high : list : Upper limit for randomization

    """
    parent_link: str
    child_link: str
    trajectory: Any
    angle: Optional[Any] = None
    low: Optional[List[float]] = None
    high: Optional[List[float]] = None


class DynamicSubGoal(SubGoal):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        schema = OmegaConf.structured(DynamicSubGoalConfig)
        config = OmegaConf.create(self._content_dict)
        self._config = OmegaConf.merge(schema, config)
        self.checkCompleteness()
        if self.type() == 'splineSubGoal':
            self._traj = SplineTrajectory(self.m(), traj=self._config.trajectory)
        elif self.type() == 'analyticSubGoal':
            self._traj = AnalyticTrajectory(self.m(), traj=self._config.trajectory)
        self._traj.concretize()
        self.checkDimensionality()

    def parentLink(self):
        return self._config.parent_link

    def childLink(self):
        return self._config.child_link

    def traj(self):
        return self._traj

    def toDict(self):
        return OmegaConf.to_container(self._config)

    def position(self, **kwargs):
        return self.evaluate(**kwargs)[0]

    def velocity(self, **kwargs):
        return self.evaluate(**kwargs)[1]

    def acceleration(self, **kwargs):
        return self.evaluate(**kwargs)[2]

    def evaluate(self, **kwargs):
        t = kwargs.get('t') if 't' in kwargs else 0.0
        return self._traj.evaluate(t=t)

    def shuffle(self):
        pass

    def angle(self):
        return self._config.angle

    def renderGym(self, viewer, rendering, **kwargs):
        coordinate_system_1 = viewer.draw_line([-3, 0], [3, 0], color=[0.0, 0.0, 0.0])
        coordinate_system_2 = viewer.draw_line([0, -3], [0, 3], color=[0.0, 0.0, 0.0])
        angle = self.angle()
        if not angle:
            angle = 0.0
        tf2 = rendering.Transform(rotation=angle)
        coordinate_system_1.add_attr(tf2)
        coordinate_system_2.add_attr(tf2)
        if self.m() == 1:
            start_point = [-10, -10.0]
            end_point = [10.0, 10.0]
            start_point[self.indices()[0]] = 0.0
            end_point[self.indices()[0]] = 0.0
            goal = viewer.draw_line(start_point, end_point, color=[0.0, 1.0, 0.0])
            translation = [0.0, 0.0]
            translation[self.indices()[0]] = self.position(t=kwargs.get('t'))[0]
            tf = rendering.Transform(translation=translation)
            goal.add_attr(tf)
            goal.add_attr(tf2)
        elif self.m() == 2:
            x = self.position(t=kwargs.get('t'))
            goal = viewer.draw_circle(self.epsilon(), color=[0.0, 1.0, 0.0])
            tf = rendering.Transform(translation=(x[0], x[1]))
            goal.add_attr(tf)
            goal.add_attr(tf2)
        else:
            raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")

    def updateBulletPosition(self, pybullet, **kwargs):
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        pos = self.position(t=t).tolist()
        if self.m() == 2 and self.indices() == [0, 1]:
            pos += [0.0]
        ori = [0, 0, 0, 1]
        pybullet.resetBasePositionAndOrientation(self._bulletId, pos, ori)

    def add2Bullet(self, pybullet, position=[0.0, 0.0, 0.0]):
        if self.m() == 2:
            basePosition = self.position() + [0.0]
        elif self.m() == 3:
            basePosition = self.position()
        else:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        rgbaColor = [0.0, 1.0, 0.0, 0.3]
        visualShapeId = pybullet.createVisualShape(pybullet.GEOM_SPHERE, rgbaColor=rgbaColor, radius=self.epsilon())
        collisionShape = -1
        baseOrientation = [0, 0, 0, 1]
        mass = 0

        self._bulletId = pybullet.createMultiBody(
                    mass,
                    collisionShape,
                    visualShapeId,
                    basePosition,
                    baseOrientation,
        )

        if self.angle():
            for i in range(3):
                self.addAxisComponent(pybullet, i, position)

    def addAxisComponent(self, pybullet, i, goal_position):
        rgbaColor = [0.0, 0.0, 0.0, 0.3]
        angles = self.angle()
        if not angles:
            angles = Quaternion([1, 0., 0, 0])
        else:
            angles = Quaternion(angles).inverse
        orientation = Quaternion([1, 0, 0, 0])
        orientation[i+1] = 1.0
        orientation = orientation.normalised
        position = np.array([goal_position[0], goal_position[1], goal_position[2]])
        orientation = angles * orientation
        orientation_array = [orientation[1], orientation[2], orientation[3], orientation[0]]
        offset_array = [np.array([0.00, 0.00, -0.05]), np.array([0.0, 0.0, 0.05]), np.array([0.0, 0.0, 0.05])]

        position += orientation.rotate(offset_array[i])
        index_map = [1, 0, 2]
        rgbaColor[index_map[i]] = 1.0
        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_CYLINDER, rgbaColor=rgbaColor,radius=0.01, length=0.1
        )
        collisionShape = -1
        mass = 0
        pybullet.createMultiBody(
                    mass,
                    collisionShape,
                    visual_shape_id,
                    position,
                    orientation_array,
        )
