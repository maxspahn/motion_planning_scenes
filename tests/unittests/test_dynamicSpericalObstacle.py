import pytest

from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle


def test_circleObstacle():
    obstDict = {
        "type": "sphere",
        "position": [0,0,0],
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    assert "dynamicSphere" == dynamicSphereObst.name()
    assert 0.2 == dynamicSphereObst.radius()
    pos_t_00 = dynamicSphereObst.position(t=0.0)
    assert pos_t_00[0] == 0.0
    assert pos_t_00[1] == 0.0
    pos_t_12 = dynamicSphereObst.position(t=1.2)
    assert pos_t_12[0] == 0.1 * 1.2
    assert pos_t_12[1] == 0.2 * 1.2


def test_splineObstacle():
    splineDict = {'degree': 2, 'controlPoints': [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], 'duration': 10}
    obstDict = {
        "type": "splineSphere",
        "position": [0,0,0],
        "geometry": {"trajectory": splineDict, "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    assert "dynamicSphere" == dynamicSphereObst.name()
    assert 0.2 == dynamicSphereObst.radius()
