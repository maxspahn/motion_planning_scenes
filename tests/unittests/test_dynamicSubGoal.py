import pytest
import numpy as np

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
    assert isinstance(dynamic_sub_goal.position(t=0), np.ndarray)
    assert isinstance(dynamic_sub_goal.evaluate(t=0), list)
    assert [0.01, 0.2] == dynamic_sub_goal.position(t=0).tolist()
    assert [1.01, 0.2] == dynamic_sub_goal.position(t=1).tolist()
    assert [0.00, 0.0] == dynamic_sub_goal.acceleration(t=0).tolist()
    assert [0.00, 0.0] == dynamic_sub_goal.acceleration(t=1).tolist()

def test_mask_selection(dynamicGoalDict):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicGoalDict)
    mask = ["position", "velocity", "epsilon", "parent_link"]
    selected_items = dynamic_sub_goal.evaluate_components(mask, 0.2)
    assert isinstance(selected_items, dict)
    assert isinstance(selected_items['position'], np.ndarray)
    assert isinstance(selected_items['velocity'], np.ndarray)
    assert isinstance(selected_items['epsilon'], float)
    assert isinstance(selected_items['parent_link'], str)
    assert selected_items['epsilon'] == 0.2
    assert list(selected_items.keys()) == ["position", "velocity", "epsilon", "parent_link"]


def test_dynamicSplineSubGoal(dynamicSplineGoalDict):
    dynamic_sub_goal = DynamicSubGoal(name="simple_dynamic_subGoal", content_dict=dynamicSplineGoalDict)
    assert "simple_dynamic_subGoal" == dynamic_sub_goal.name()
    assert 0.2 == dynamic_sub_goal.epsilon()

