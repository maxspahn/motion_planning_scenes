import pytest

from MotionPlanningGoal.goalComposition import GoalComposition, MultiplePrimeGoalsError


def test_goalComposition():
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


def test_goalCompositionMulti():
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
    goalComposition = GoalComposition(
        name="example_static_subGoal", contentDict=goalDict
    )
    subGoal1 = goalComposition.getGoalByName("subgoal1")
    assert "subgoal1" == subGoal1.name()
    assert subGoal1.position() == [-0.21, 0.2]


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
