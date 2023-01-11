import pytest
import numpy as np

from mpscenes.goals.goal_composition import GoalComposition
from mpscenes.common.errors import MultiplePrimeGoalsError


def test_goal_composition_single():
    goal_dict = {
        "subgoal0": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        }
    }
    goal_composition = GoalComposition(
        name="example_static_subGoal", content_dict=goal_dict
    )
    sub_goal_0 = goal_composition.primary_goal()
    assert "subgoal0" == sub_goal_0.name()
    assert sub_goal_0.indices() == [0, 1]
    assert sub_goal_0.parent_link() == 0
    assert isinstance(sub_goal_0.parent_link(), int)


@pytest.fixture
def multi_goal_dict():
    goal_dict = {
        "subgoal0": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
        "subgoal1": {
            "weight": 5.0,
            "is_primary_goal": False,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [-0.21, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
    }
    return goal_dict


def test_goal_composition_multi(multi_goal_dict):
    goal_composition = GoalComposition(
        name="example_static_subGoal", content_dict=multi_goal_dict
    )
    sub_goal_1 = goal_composition.get_goal_by_name("subgoal1")
    assert "subgoal1" == sub_goal_1.name()
    assert isinstance(sub_goal_1.position(), np.ndarray)
    assert sub_goal_1.position().tolist() == [-0.21, 0.2]
    sub_goals = goal_composition.sub_goals()
    assert len(sub_goals) == 2


def test_shuffleGoalComposition(multi_goal_dict):
    goal_composition = GoalComposition(
        name="example_static_subGoal", content_dict=multi_goal_dict
    )
    # verification that returns are actually only pointers
    sub_goal_1 = goal_composition.get_goal_by_name('subgoal1')
    assert sub_goal_1.position().tolist() == [-0.21, 0.2]
    goal_composition.shuffle()
    assert goal_composition.get_goal_by_name('subgoal1').position().tolist() != [-0.21, 0.2]
    assert sub_goal_1.position().tolist() != [-0.21, 0.2]


def test_errorMultiplePrimeGoals():
    goalDict = {
        "subgoal0": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
        "subgoal1": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [-0.21, 0.2],
            "epsilon": 0.2,
            "type": "staticSubGoal",
        },
    }
    with pytest.raises(MultiplePrimeGoalsError) as e_info:
        GoalComposition(name="example_static_subGoal", content_dict=goalDict)
