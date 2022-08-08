import pytest

from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal


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


def test_dynamicSubGoal(dynamicGoalDict):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicGoalDict)
    assert "simple_dynamic_subGoal" == dynamic_sub_goal.name()
    assert 0.2 == dynamic_sub_goal.epsilon()
    assert [0.01, 0.2] == dynamic_sub_goal.position(t=0).tolist()
    assert [1.01, 0.2] == dynamic_sub_goal.position(t=1).tolist()

def test_dynamicSplineSubGoal(dynamicSplineGoalDict):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicSplineGoalDict)
    assert "simple_dynamic_subGoal" == dynamic_sub_goal.name()
    assert 0.2 == dynamic_sub_goal.epsilon()

