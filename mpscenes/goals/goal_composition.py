from mpscenes.common.component import MPComponent
from mpscenes.goals.sub_goal import SubGoal
from mpscenes.goals.sub_goal_creator import SubGoalCreator
from mpscenes.common.errors import JointSpaceGoalsNotSupportedError

import yaml
import logging
from omegaconf import OmegaConf

from mpscenes.common.errors import MultiplePrimeGoalsError


class GoalComposition(MPComponent):
    def __init__(self, **kwargs):
        if "content_dict" in kwargs and "name" in kwargs:
            self._content_dict = kwargs.get("content_dict")
            self._name = kwargs.get("name")
        elif "file_name" in kwargs:
            with open(kwargs.get("file_name"), "r") as stream:
                self._content_dict = yaml.safe_load(stream)
            self._name = self._content_dict["name"]
            del self._content_dict["name"]
        self._config = OmegaConf.create(self._content_dict)
        self._primary_goal_index = -1
        self._sub_goals = []
        self._sub_goal_creator = SubGoalCreator()
        self.parse_sub_goals()

    def parse_sub_goals(self):
        for sub_goal_name in self._config.keys():
            sub_goal_type = self._config[sub_goal_name].type
            sub_goal_dict = self._config[sub_goal_name]
            sub_goal = self._sub_goal_creator.create_sub_goal(
                sub_goal_type, sub_goal_name, sub_goal_dict
            )
            if sub_goal.is_primary_goal():
                if self._primary_goal_index >= 0:
                    raise MultiplePrimeGoalsError(
                        "There are multiple prime goals. Using first prime goal"
                    )
                else:
                    self._primary_goal_index = len(self._sub_goals)
            self._sub_goals.append(sub_goal)

    def primary_goal(self):
        return self.get_goal_by_index(self._primary_goal_index)

    def sub_goals(self):
        return self._sub_goals

    def get_goal_by_name(self, name) -> SubGoal:
        for sub_goal in self._sub_goals:
            if sub_goal.name() == name:
                return sub_goal

    def get_goal_by_index(self, index):
        return self._sub_goals[index]

    def evaluate(self, **kwargs):
        evals = []
        for sub_goal in self._sub_goals:
            evals += sub_goal.evaluate(**kwargs)
        return evals

    def dict(self):
        composition_dict = {}
        for sub_goal in self._sub_goals:
            composition_dict[sub_goal.name()] = sub_goal.dict()
        return composition_dict

    def shuffle(self):
        for sub_goal in self._sub_goals:
            sub_goal.shuffle()

    def render_gym(self, viewer, rendering, **kwargs):
        for sub_goal in self._sub_goals:
            try:
                sub_goal.render_gym(viewer, rendering, **kwargs)
            except JointSpaceGoalsNotSupportedError as _:
                logging.warn("Skipping visualization of joint space goal.")

    def add_to_bullet(self, pybullet):
        for sub_goal in self._sub_goals:
            try:
                sub_goal.add_to_bullet(
                    pybullet, position=self.primary_goal().position()
                )
            except JointSpaceGoalsNotSupportedError as _:
                print("Skipping visualization of joint space goal.")

    def update_bullet_position(self, pybullet, **kwargs):
        self.primary_goal().update_bullet_position(pybullet, **kwargs)
