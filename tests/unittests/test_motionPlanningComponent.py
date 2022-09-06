from MotionPlanningSceneHelpers.motionPlanningComponent import MotionPlanningComponent
from omegaconf import OmegaConf
from dataclasses import dataclass


@dataclass
class MotionPlanningComponentConfig():
    """
    test data class for motion planning components.
    """
    parameter1: float
    parameter2: bool = True

def test_add_required_keys():
    required_keys = ["key1", "key2", "key3"]

    required_keys_dict = {
            "geometry":
            ["length", "width", "radius"]
            }
    required_keys_dict_answer = [
            "geometry.length",
            "geometry.width",
            "geometry.radius"
            ]

    required_keys_nested_dict = {"geometry": [
        "diameter",
        {"head": ["l", "w", "h"]},
        {"body": ["length", "width", "radius"]},
        ]
        }
    required_keys_nested_dict_answer = ["geometry.diameter",
        "geometry.head.l",
        "geometry.head.w",
        "geometry.head.h",
        "geometry.body.length",
        "geometry.body.width",
        "geometry.body.radius"
        ]

    required_keys_list = [required_keys,
            required_keys_dict,
            required_keys_nested_dict]
    expected_keys_list = [required_keys,
            required_keys_dict_answer,
            required_keys_nested_dict_answer]

    # add required keys and check if the expected keys are returned
    for (required_keys, expected_keys) in zip(required_keys_list,
            expected_keys_list):

        motion_pc_dict = {
            "parameter1": 15.03,
            "parameter2": False
        }

        schema = OmegaConf.structured(MotionPlanningComponentConfig)
        motion_pc = MotionPlanningComponent(schema,
                name="motion_pc",
                content_dict=motion_pc_dict)

        motion_pc.add_required_keys(required_keys)
        assert motion_pc._required_keys == expected_keys

def add_unallowed_required_keys():

    motion_pc_dict = {
        "parameter1": 15.03,
        "parameter2": False
    }

    schema = OmegaConf.structured(MotionPlanningComponentConfig)
    motion_pc = MotionPlanningComponent(schema,
            name="motion_pc",
            content_dict=motion_pc_dict)
    with pytest.raises(ValueError):
        motion_pc.add_required_keys(["string-1", "string.2", "string3"])

    with pytest.raises(TypeError):
        motion_pc.add_required_keys(["string1", 2, "string3"])
