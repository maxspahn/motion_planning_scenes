from omegaconf.errors import MissingMandatoryValue
import pytest

from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.sphereObstacle import SphereObstacleMissmatchDimensionError
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


def test_circleObstacle():
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    assert "simpleSphere" == sphereObst.name()
    assert [0.1, 0.2] == sphereObst.position()
    assert 0.2 == sphereObst.radius()
    assert 2 == sphereObst.dimension()

def test_sphereObstacle():
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    assert "simpleSphere" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert 0.2 == sphereObst.radius()
    assert 3 == sphereObst.dimension()


def test_errorRaiseIncompleteDict():
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2]}}
    with pytest.raises(ComponentIncompleteError):
        sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
        sphereObst.radius()

def test_saving_obstacle():
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    sphereObst.shuffle()
    obst_dict_after = sphereObst.dict()
    assert isinstance(obst_dict_after, dict)
    assert obst_dict_after['geometry']['position'][0] != 0.1

