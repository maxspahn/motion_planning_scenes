import pytest

from MotionPlanningGoal.staticSubGoal import StaticSubGoal

from MotionPlanningGoal.subGoal import SubGoalMissmatchDimensionError
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


@pytest.fixture
def simpleGoalDict():
    goalDict = {
        "m": 2,
        "w": 5.0,
        "prime": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "staticSubGoal",
    }
    return goalDict


def test_staticSubGoal(simpleGoalDict):
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", contentDict=simpleGoalDict)
    assert "simple_static_subGoal" == simpleStaticSubGoal.name()
    assert [0.01, 0.2] == simpleStaticSubGoal.position()
    assert 0.2 == simpleStaticSubGoal.epsilon()


def test_shuffleGoal(simpleGoalDict):
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", contentDict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    assert [0.01, 0.2] != simpleStaticSubGoal.position()
    assert simpleStaticSubGoal.position()[0] >= -1
    assert simpleStaticSubGoal.position()[0] <= 1
    assert simpleStaticSubGoal.position()[1] >= -1
    assert simpleStaticSubGoal.position()[1] <= 1
    # add limits to goalDict
    simpleGoalDict['low'] = [-2, -2]
    simpleGoalDict['high'] = [-1, -1]
    simpleStaticSubGoal = StaticSubGoal(name="simple_static_subGoal", contentDict=simpleGoalDict)
    simpleStaticSubGoal.shuffle()
    assert [0.01, 0.2] != simpleStaticSubGoal.position()
    assert simpleStaticSubGoal.position()[0] >= -2
    assert simpleStaticSubGoal.position()[0] <= -1
    assert simpleStaticSubGoal.position()[1] >= -2
    assert simpleStaticSubGoal.position()[1] <= -1


def test_errorRaiseIncompleteDict():
    goalDict = {
        "w": 5.0,
        "prime": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
    }
    with pytest.raises(ComponentIncompleteError) as e_info:
        StaticSubGoal(name="example_static_subGoal", contentDict=goalDict)


def test_errorRaiseMissmatichDimension():
    goalDict = {
        "m": 1,
        "w": 5.0,
        "prime": True,
        "epsilon": 0.2,
        "indices": [0],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "type": "staticSubGoal",
    }
    with pytest.raises(SubGoalMissmatchDimensionError) as e_info:
        StaticSubGoal(name="example_static_subGoal", contentDict=goalDict)
