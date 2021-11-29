import pytest
from MotionPlanningGoal.subGoalCreator import SubGoalCreator, UnknownSubGoalType

def test_subGoalCreator():
    goalDict = {'m': 2, 'w': 5.0, 'prime': True, 'indices': [0, 1], 'parent_link': 0, 'child_link': 3, 'desired_position': [0.01, 0.2], 'epsilon': 0.2, 'type': 'staticSubGoal'}
    subGoalCreator = SubGoalCreator()
    subgoal0 = subGoalCreator.createSubGoal('staticSubGoal', 'subgoal0', goalDict)
    assert subgoal0.indices() == [0, 1]

def test_unknownSubGoalTypeError():
    goalDict = {'m': 2, 'w': 5.0, 'prime': True, 'indices': [0, 1], 'parent_link': 0, 'child_link': 3, 'desired_position': [0.01, 0.2], 'epsilon': 0.2, 'type': 'staticSubGoal'}
    subGoalCreator = SubGoalCreator()
    with pytest.raises(UnknownSubGoalType):
        subGoalCreator.createSubGoal('stateSubGoal', 'subgoal0', goalDict)
