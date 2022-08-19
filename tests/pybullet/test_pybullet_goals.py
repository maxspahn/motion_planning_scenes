import pytest
import time

import pybullet as p
import pybullet_data

from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal
from MotionPlanningGoal.staticJointSpaceSubGoal import StaticJointSpaceSubGoal, JointSpaceGoalsNotSupportedError

no_gui = True

@pytest.fixture
def simpleGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1, 2],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2, 1.0],
        "epsilon": 0.2,
        "type": "staticSubGoal",
    }
    return goalDict

@pytest.fixture
def simpleJointSpaceGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1, 2],
        "desired_position": [0.01, 0.2, 1.0],
        "epsilon": 0.2,
        "type": "staticJointSpaceSubGoal",
    }
    return goalDict

@pytest.fixture
def dynamicGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "trajectory": ["0.01 + t*1", "0.2"],
        "epsilon": 0.2,
        "type": "analyticSubGoal",
    }
    return goalDict

@pytest.fixture
def dynamicSplineGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "trajectory": {'degree': 2, 'controlPoints': [[0.1, 0.0], [1.0, 1.0], [1.0, 2.0]], 'duration': 10},
        "epsilon": 0.2,
        "type": "splineSubGoal",
    }
    return goalDict


@pytest.fixture
def bullet():
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    return p

@pytest.fixture
def bullet_gui():
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    return p

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_add_static_sub_goal_gui(simpleGoalDict, bullet_gui):
    static_sub_goal = StaticSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    static_sub_goal.add_to_bullet(bullet_gui)
    for _ in range(10):
        bullet_gui.stepSimulation()
        time.sleep(0.1)
    bullet_gui.disconnect()

def test_add_static_sub_goal(simpleGoalDict, bullet):
    static_sub_goal = StaticSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    static_sub_goal.add_to_bullet(bullet)
    for _ in range(10):
        bullet.stepSimulation()
    bullet.disconnect()

def test_add_joint_space_goal(simpleJointSpaceGoalDict, bullet):
    static_sub_goal = StaticJointSpaceSubGoal(name="simple_static_subGoal", content_dict=simpleJointSpaceGoalDict)
    with pytest.raises(JointSpaceGoalsNotSupportedError):
        static_sub_goal.add_to_bullet(bullet)
        for _ in range(10):
            bullet.stepSimulation()
    bullet.disconnect()


def test_dynamicSubGoal(dynamicGoalDict, bullet):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicGoalDict)
    dynamic_sub_goal.add_to_bullet(bullet)
    for i in range(10):
        bullet.stepSimulation()
        dynamic_sub_goal.update_bullet_position(bullet, t=i/100)
        time.sleep(i/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_dynamicSubGoal_gui(dynamicGoalDict, bullet_gui):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicGoalDict)
    dynamic_sub_goal.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamic_sub_goal.update_bullet_position(bullet_gui, t=i/100)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_dynamicSplineSubGoal(dynamicSplineGoalDict, bullet):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicSplineGoalDict)
    dynamic_sub_goal.add_to_bullet(bullet)
    for i in range(10):
        bullet.stepSimulation()
        dynamic_sub_goal.update_bullet_position(bullet, t=i/100)
        time.sleep(i/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_dynamicSplineSubGoal_gui(dynamicSplineGoalDict, bullet_gui):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicSplineGoalDict)
    dynamic_sub_goal.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamic_sub_goal.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

