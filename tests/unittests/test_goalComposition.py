import pytest

from MotionPlanningGoal.goalComposition import GoalComposition, MultiplePrimeGoalsError


def test_goalComposition_single():
    goalDict = {
        "subgoal0": {
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
    }
    goalComposition = GoalComposition(
        name="example_static_subGoal", contentDict=goalDict
    )
    subGoal0 = goalComposition.primeGoal()
    assert "subgoal0" == subGoal0.name()
    assert subGoal0.indices() == [0, 1]
    assert subGoal0.parentLink() == 0
    assert isinstance(subGoal0.parentLink(), int)


@pytest.fixture
def multiGoalDict():
    goalDict = {
        "subgoal0": {
            "m": 2,
            "w": 5.0,
            "prime": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
        "subgoal1": {
            "m": 2,
            "w": 5.0,
            "prime": False,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [-0.21, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
    }
    return goalDict


def test_goalCompositionMulti(multiGoalDict):
    goalComposition = GoalComposition(
        name="example_static_subGoal", contentDict=multiGoalDict
    )
    subGoal1 = goalComposition.getGoalByName("subgoal1")
    assert "subgoal1" == subGoal1.name()
    assert subGoal1.position() == [-0.21, 0.2]
    subGoals = goalComposition.subGoals()
    assert len(subGoals) == 2


def test_shuffleGoalComposition(multiGoalDict):
    goalComposition = GoalComposition(
        name="example_static_subGoal", contentDict=multiGoalDict
    )
    # verification that returns are actually only pointers
    subGoal1 = goalComposition.getGoalByName('subgoal1')
    assert subGoal1.position() == [-0.21, 0.2]
    goalComposition.shuffle()
    assert goalComposition.getGoalByName('subgoal1').position() != [-0.21, 0.2]
    assert subGoal1.position() != [-0.21, 0.2]


def test_errorMultiplePrimeGoals():
    goalDict = {
        "subgoal0": {
            "m": 2,
            "w": 5.0,
            "prime": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
        "subgoal1": {
            "m": 2,
            "w": 5.0,
            "prime": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [-0.21, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
    }
    with pytest.raises(MultiplePrimeGoalsError) as e_info:
        GoalComposition(name="example_static_subGoal", contentDict=goalDict)
