import pytest

from MotionPlanningEnv.cylinderObstacle import CylinderObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError

def test_cylinder_obstacle():
    obst_dict = {"type": "cylinder", "position": [0.1, 0.2, 0.4],
            "geometry": {"radius": 0.5, "height": 0.5}}
    sphere_obst = CylinderObstacle(name="simpleCylinder",
             content_dict=obst_dict)
    assert "simpleCylinder" == sphere_obst.name()
    assert [0.1, 0.2, 0.4] == sphere_obst.position()
    assert 0.5 == sphere_obst.radius()
    assert 0.5 == sphere_obst.height()


def test_error_raise_incomplete_dict():
    obs_dict = {"type": "cylinder", "position": [0.1, 0.2],
            "geometry": {"radius": 0.9}}
    with pytest.raises(ComponentIncompleteError):
        CylinderObstacle(name="simpleCylinder", content_dict=obs_dict)

def test_radius_height_incorrect_type():
    not_floats = [[0.5], [0.5, 0,5], None, {}]

    for nofloat in not_floats:

        obs_dict = {"type": "cylinder", "position": [0.1, 0.2, 0.5],
                "geometry": {"radius": nofloat, "height": 0.5}}
        with pytest.raises(ValueError) as _:
            CylinderObstacle(name="simpleCylinder", content_dict=obs_dict)
        obs_dict = {"type": "cylinder", "position": [0.1, 0.2, 0.5],
                "geometry": {"radius": 0.5, "height": nofloat}}
        with pytest.raises(ValueError) as _:
            CylinderObstacle(name="simpleCylinder", content_dict=obs_dict)
