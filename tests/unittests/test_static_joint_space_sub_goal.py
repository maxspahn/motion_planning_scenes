import pytest
from omegaconf.errors import MissingMandatoryValue

from mpscenes.goals.static_joint_space_sub_goal import StaticJointSpaceSubGoal
from mpscenes.common.errors import MissmatchDimensionError, ComponentIncompleteError


@pytest.fixture
def simpleGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "desired_position": [0.01, 0.2, 0.5],
        "epsilon": 0.2,
        "type": "staticJointSpaceSubGoal",
        "indices": [0, 1, 2], 
    }
    return goalDict


def test_staticSubGoal(simpleGoalDict):
    simpleStaticSubGoal = StaticJointSpaceSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    assert "simple_static_subGoal" == simpleStaticSubGoal.name()
    assert [0.01, 0.2, 0.5] == simpleStaticSubGoal.position()
    assert 0.2 == simpleStaticSubGoal.epsilon()
    assert "staticJointSpaceSubGoal" == simpleStaticSubGoal.type()

def test_shuffleGoal(simpleGoalDict):
    simpleStaticSubGoal = StaticJointSpaceSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    assert [0.01, 0.2, 0.5] != simpleStaticSubGoal.position()
    assert simpleStaticSubGoal.position()[0] >= -1
    assert simpleStaticSubGoal.position()[0] <= 1
    assert simpleStaticSubGoal.position()[1] >= -1
    assert simpleStaticSubGoal.position()[1] <= 1
    # add limits to goalDict
    simpleGoalDict['low'] = [-2, -2, -2]
    simpleGoalDict['high'] = [-1, -1, 0]
    simpleStaticSubGoal = StaticJointSpaceSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    assert [0.01, 0.2, 0.5] != simpleStaticSubGoal.position()
    assert simpleStaticSubGoal.position()[0] >= -2
    assert simpleStaticSubGoal.position()[0] <= -1
    assert simpleStaticSubGoal.position()[1] >= -2
    assert simpleStaticSubGoal.position()[1] <= -1
    assert simpleStaticSubGoal.position()[2] >= -2
    assert simpleStaticSubGoal.position()[2] <= 0

def test_saving_sub_goal(simpleGoalDict):
    simpleStaticSubGoal = StaticJointSpaceSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    goal_dict_after =simpleStaticSubGoal.dict()
    assert isinstance(goal_dict_after, dict)
    assert goal_dict_after['desired_position'][0] != 0.01


def test_errorRaiseIncompleteDict():
    goalDict = {
        "is_primary_goal": True,
        "indices": [0, 1],
        "desired_position": [0.01, 0.2],
    }
    with pytest.raises(MissingMandatoryValue) as e_info:
        static_sub_goal = StaticJointSpaceSubGoal(name="example_static_subGoal", content_dict=goalDict)
        weight = static_sub_goal.weight()


def test_errorRaiseMissmatichDimension():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "epsilon": 0.2,
        "indices": [0],
        "desired_position": [0.01, 0.2],
        "type": "staticJointSpaceSubGoal",
    }
    with pytest.raises(MissmatchDimensionError) as e_info:
        StaticJointSpaceSubGoal(name="example_static_subGoal", content_dict=goalDict)
