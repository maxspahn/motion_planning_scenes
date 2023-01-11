from mpscenes.goals.static_sub_goal import StaticSubGoal
from mpscenes.goals.static_joint_space_sub_goal import StaticJointSpaceSubGoal
from mpscenes.goals.dynamic_sub_goal import DynamicSubGoal
from mpscenes.common.errors import UnknownSubGoalType


class SubGoalCreator(object):
    def __init__(self):
        pass

    def create_sub_goal(self, sub_goal_type, name, content_dict):
        if sub_goal_type == "staticSubGoal":
            return StaticSubGoal(name=name, content_dict=content_dict)
        elif sub_goal_type in ("analyticSubGoal", "splineSubGoal"):
            return DynamicSubGoal(name=name, content_dict=content_dict)
        elif sub_goal_type == "staticJointSpaceSubGoal":
            return StaticJointSpaceSubGoal(name=name, content_dict=content_dict)
        else:
            raise UnknownSubGoalType(
                f"SubGoalType {sub_goal_type} is not known"
            )
