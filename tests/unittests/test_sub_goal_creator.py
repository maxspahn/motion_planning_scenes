import pytest

from mpscenes.goals.static_joint_space_sub_goal import StaticJointSpaceSubGoal
from mpscenes.goals.sub_goal_creator import SubGoalCreator
from mpscenes.common.errors import UnknownSubGoalType


def test_sub_goal_creator():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "staticSubGoal",
    }
    sub_goal_creator = SubGoalCreator()
    subgoal0 = sub_goal_creator.create_sub_goal(
        "staticSubGoal", "subgoal0", goal_dict
    )
    assert subgoal0.indices() == [0, 1]


def test_sub_goal_creator_joint_space_goal():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "staticJointSpaceSubGoal",
    }
    sub_goal_creator = SubGoalCreator()
    sub_goal_0 = sub_goal_creator.create_sub_goal(
        "staticJointSpaceSubGoal", "subgoal0", goal_dict
    )
    assert sub_goal_0.type() == "staticJointSpaceSubGoal"
    assert isinstance(sub_goal_0, StaticJointSpaceSubGoal)


def test_unknown_sub_goal_type_error():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "staticSubGoal",
    }
    sub_goal_creator = SubGoalCreator()
    with pytest.raises(UnknownSubGoalType):
        sub_goal_creator.create_sub_goal("stateSubGoal", "subgoal0", goal_dict)
