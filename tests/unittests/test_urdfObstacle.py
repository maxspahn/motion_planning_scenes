import pytest

from MotionPlanningEnv.urdfObstacle import UrdfObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


def test_urdfObstacle():
    obstDict = {
        "dim": 3,
        "type": "sphere",
        "geometry": {"position": [0.1, 0.2, 0.4]},
        "urdf": "sphere_015.urdf",
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", contentDict=obstDict)
    assert "simpleUrdf" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert "sphere_015.urdf" == sphereObst.urdf()


def test_yamlLoad():
    yamlFile = 'yamlExamples/urdfSphere.yaml'
    sphereObst = UrdfObstacle(fileName=yamlFile)
    assert "simpleUrdf" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert "sphere_015.urdf" == sphereObst.urdf()
