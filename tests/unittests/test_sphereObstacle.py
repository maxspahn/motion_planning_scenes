from omegaconf.errors import MissingMandatoryValue
import pytest

from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.sphereObstacle import SphereObstacleMissmatchDimensionError
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


def test_circleObstacle():
    obstDict = {'type': 'sphere', 'position': [0.1, 0.2], 'geometry': {'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    assert "simpleSphere" == sphereObst.name()
    assert [0.1, 0.2] == sphereObst.position()
    assert 0.2 == sphereObst.radius()
    assert 2 == sphereObst.dimension()

def test_sphereObstacle():
    obstDict = {'type': 'sphere', 'position': [0.1, 0.2, 0.4], 'geometry': {'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    assert "simpleSphere" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert 0.2 == sphereObst.radius()
    assert 3 == sphereObst.dimension()


def test_errorRaiseIncompleteDict():
    obstDict = {'type': 'sphere', 'position': [0.1, 0.2], 'geometry': {}}
    with pytest.raises(MissingMandatoryValue):
        sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
        sphereObst.radius()

def test_saving_obstacle():
    obstDict = {'type': 'sphere', 'position': [0.1, 0.2, 0.4], 'geometry': {'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    sphereObst.shuffle()
    obst_dict_after = sphereObst.dict()
    assert isinstance(obst_dict_after, dict)
    assert obst_dict_after['position'][0] != 0.1

