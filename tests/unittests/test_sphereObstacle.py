import pytest

from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.sphereObstacle import SphereObstacleMissmatchDimensionError
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


def test_circleObstacle():
    obstDict = {'dim': 2, 'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    sphereObst = SphereObstacle('simpleSphere', obstDict)
    assert "simpleSphere" == sphereObst.name()
    assert [0.1, 0.2] == sphereObst.position()
    assert 0.2 == sphereObst.radius()

def test_sphereObstacle():
    obstDict = {'dim': 3, 'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle('simpleSphere', obstDict)
    assert "simpleSphere" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert 0.2 == sphereObst.radius()


def test_errorRaiseIncompleteDict():
    obstDict = {'dim': 2, 'type': 'sphere', 'geometry': {'position': [0.1, 0.2]}}
    with pytest.raises(ComponentIncompleteError):
        sphereObst = SphereObstacle('simpleSphere', obstDict)


def test_errorRaiseMissmatichDimension():
    obstDict = {'dim': 3, 'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    with pytest.raises(SphereObstacleMissmatchDimensionError) as e_info:
        sphereObst = SphereObstacle('simpleSphere', obstDict)

