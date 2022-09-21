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
