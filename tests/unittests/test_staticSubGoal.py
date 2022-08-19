import pytest

from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.subGoal import SubGoalMissmatchDimensionError

from omegaconf.errors import MissingMandatoryValue


@pytest.fixture
def simpleGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "staticSubGoal",
    }
    return goalDict


def test_staticSubGoal(simpleGoalDict):
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    assert "simple_static_subGoal" == simpleStaticSubGoal.name()
    assert [0.01, 0.2] == simpleStaticSubGoal.position()
    assert 0.2 == simpleStaticSubGoal.epsilon()


def test_shuffleGoal(simpleGoalDict):
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    assert [0.01, 0.2] != simpleStaticSubGoal.position()
    assert simpleStaticSubGoal.position()[0] >= -1
    assert simpleStaticSubGoal.position()[0] <= 1
    assert simpleStaticSubGoal.position()[1] >= -1
    assert simpleStaticSubGoal.position()[1] <= 1
    # add limits to goalDict
    simpleGoalDict['low'] = [-2, -2]
    simpleGoalDict['high'] = [-1, -1]
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    assert [0.01, 0.2] != simpleStaticSubGoal.position()
    assert simpleStaticSubGoal.position()[0] >= -2
    assert simpleStaticSubGoal.position()[0] <= -1
    assert simpleStaticSubGoal.position()[1] >= -2
    assert simpleStaticSubGoal.position()[1] <= -1

def test_saving_sub_goal(simpleGoalDict):
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    goal_dict_after =simpleStaticSubGoal.dict()
    assert isinstance(goal_dict_after, dict)
    assert goal_dict_after['desired_position'][0] != 0.01


def test_errorRaiseIncompleteDict():
    goalDict = {
        "weight": 5.0,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
    }
    with pytest.raises(MissingMandatoryValue) as e_info:
        static_sub_goal = StaticSubGoal(name="example_static_subGoal", content_dict=goalDict)
        is_primary_gaol = static_sub_goal.is_primary_goal()


def test_errorRaiseMissmatichDimension():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "epsilon": 0.2,
        "indices": [0],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "type": "staticSubGoal",
    }
    with pytest.raises(SubGoalMissmatchDimensionError) as e_info:
        StaticSubGoal(name="example_static_subGoal", content_dict=goalDict)
