from typing import List, Dict

import yaml
import numpy as np
from omegaconf import OmegaConf

from mpscenes.common.errors import MultiplePrimeGoalsError
from mpscenes.common.component import MPComponent
from mpscenes.goals.sub_goal import SubGoal
from mpscenes.goals.sub_goal_creator import SubGoalCreator


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

    def parse_sub_goals(self) -> None:
        """
        Parse and create sub-goals based on the configuration.

        Iterates through the sub-goal configurations in the internal
        configuration dictionary and creates sub-goals using the specified
        types and parameters. The created sub-goals are stored in the
        `_sub_goals` list. If a sub-goal is marked as a primary goal, it sets
        the `_primary_goal_index` accordingly.

        Raises
        ------
        MultiplePrimeGoalsError
            If more than one sub-goal is marked as a primary goal.
        """

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

    def primary_goal(self) -> SubGoal:
        """Get primary goal of goal composition."""
        return self.get_goal_by_index(self._primary_goal_index)

    def sub_goals(self) -> List[SubGoal]:
        """Get list of all subgoals."""
        return self._sub_goals

    def get_goal_by_name(self, name: str) -> SubGoal:
        """Get subgoal by name."""
        for sub_goal in self._sub_goals:
            if sub_goal.name() == name:
                return sub_goal

    def get_goal_by_index(self, index: int) -> SubGoal:
        """Get subgoal by index."""
        return self._sub_goals[index]

    def evaluate(self, **kwargs) -> List[List[np.ndarray]]:
        """
        Evaluates all subgoals and returns them as a list.

        Evaluations containts position, velocity and acceleration.
        """
        evals = []
        for sub_goal in self._sub_goals:
            evals += sub_goal.evaluate(**kwargs)
        return evals

    def dict(self) -> Dict[str, dict]:
        composition_dict = {}
        for sub_goal in self._sub_goals:
            composition_dict[sub_goal.name()] = sub_goal.dict()
        return composition_dict

    def shuffle(self) -> None:
        for sub_goal in self._sub_goals:
            sub_goal.shuffle()

