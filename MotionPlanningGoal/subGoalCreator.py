from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.staticJointSpaceSubGoal import StaticJointSpaceSubGoal
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal


class UnknownSubGoalType(Exception):
    pass


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
