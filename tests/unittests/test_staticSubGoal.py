import pytest

from MotionPlanningGoal.staticSubGoal import StaticSubGoal

from MotionPlanningGoal.subGoal import SubGoalMissmatchDimensionError
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


def test_staticSubGoal():
    goalDict = {'m': 2, 'w': 5.0, 'prime': True, 'indices': [0, 1], 'parent_link': 0, 'child_link': 3, 'desired_position': [0.01, 0.2]}
    staticSubGoal = StaticSubGoal(name="example_static_subGoal", contentDict=goalDict)
    assert "example_static_subGoal" == staticSubGoal.name()
    assert [0.01, 0.2] == staticSubGoal.evaluate(0.1)
    assert [0.01, 0.2] == staticSubGoal.getDesiredPosition()


def test_errorRaiseIncompleteDict():
    goalDict = {'w': 5.0, 'prime': True, 'indices': [0, 1], 'parent_link': 0, 'child_link': 3, 'desired_position': [0.01, 0.2]}
    with pytest.raises(ComponentIncompleteError) as e_info:
        StaticSubGoal(name="example_static_subGoal", contentDict=goalDict)


def test_errorRaiseMissmatichDimension():
    goalDict = {'m': 1, 'w': 5.0, 'prime': True, 'indices': [0], 'parent_link': 0, 'child_link': 3, 'desired_position': [0.01, 0.2]}
    with pytest.raises(SubGoalMissmatchDimensionError) as e_info:
        StaticSubGoal(name="example_static_subGoal", contentDict=goalDict)

