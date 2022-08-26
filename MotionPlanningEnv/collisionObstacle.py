from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent

from typing import List
from dataclasses import dataclass

@dataclass
class CollisionObstacleConfig:
    """
    Configuration dataclass for an obstacle.
    This configuration class holds information about the dimension and the type
    of collision obstacle.

    Parameters:
    ------------
    position: list: [x,y,z] Position of the obstacle
    type : str : Type of the obstacle
    """
    position: List[float]
    type: str

class CollisionObstacle(MotionPlanningComponent):

    def __init__(self, schema, **kwargs):
        self._required_keys = [
            "position",
            "type",
        ]
        super().__init__(schema, **kwargs)

        self.sanitize_orientation()
        self.sanitize_color()
        self.sanitize_mass()

    def sanitize_orientation(self):
        if "orientation" in self._content_dict:
            if len(self._content_dict["orientation"]) != 4:
                raise ValueError(f"""
                        incorrect orientation shape: {len(
                        self._content_dict['orientation'])}, which should be (4,)
                        """)

    def sanitize_color(self):
        if "color" in self._content_dict:
            if len(self._content_dict["color"]) != 4:
                raise ValueError(f"""
                        incorrect color shape: {len(
                        self._content_dict['color'])}, which should be (4,)
                        """)

    def sanitize_mass(self):
        if "mass" in self._content_dict:
            if self._content_dict["mass"] <= 0:
                raise ValueError(f"""
                        negative mass: 
                        {self._content_dict['mass']}, which should positive
                        """)

    def dimension(self):
        return len(self._config.position)

    def type(self):
        return self._config.type

    def position(self):
        return self._config.position

    def velocity(self):
        raise NotImplementedError

    def acceleration(self):
        raise NotImplementedError

    def movable(self):
        return self._config.movable

    def mass(self):
        return self._config.mass

    def color(self):
        return self._config.color

    def geometry(self):
        return self._config.geometry

    def orientation(self):
        if "orientation" in self._config.orientation:
            return self._config.orientation()

    def update_bullet_position(self, pybullet, **kwargs):
        pass
