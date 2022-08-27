import pytest

# because it is not possbile to directly
# create an collisionObstacle boxObstacle is used.
from MotionPlanningEnv.boxObstacle import BoxObstacle

def test_orientation_incorrect_type():
    obst_dict = {"type": "box",
            "position": [0.1, 0.2, 0.5],
            "geometry": {"length": 0.5, "width": 0.5, "height": 0.5},
            "orientation": ["string", 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name="simpleBox", content_dict=obst_dict)

def test_orientation_incorrect_shape():
    obst_dict = {"type": "box",
            "position": [0.1, 0.2, 0.5],
            "geometry": {"length": 0.5, "width": 0.5, "height": 0.5},
            "orientation": [1, 0.4, 0.8, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name="simpleBox", content_dict=obst_dict)

def test_color_incorrect_shape():
    obst_dict = {"type": "box",
            "position": [0.1, 0.2, 0.5],
            "geometry": {"length": 0.5, "width": 0.5, "height": 0.5},
            "color": [1, 0, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name="simpleBox", content_dict=obst_dict)

def test_negative_mass():
    obst_dict = {"type": "box",
            "position": [0.1, 0.2, 0.5],
            "geometry": {"length": 0.5, "width": 0.5, "height": 0.5},
            "mass": -15,
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name="simpleBox", content_dict=obst_dict)

def test_color_incorrect_type():
    obst_dict = {"type": "box",
            "position": [0.1, 0.2, 0.5],
            "geometry": {"length": 0.5, "width": 0.5, "height": 0.5},
            "color": ["string", 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name="simpleBox", content_dict=obst_dict)

