import os

from MotionPlanningEnv.urdfObstacle import UrdfObstacle


def test_urdfObstacle():
    obstDict = {
        "dim": 3,
        "type": "sphere",
        "geometry": {"position": [0.1, 0.2, 0.4]},
        "urdf": "duck.urdf",
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", contentDict=obstDict)
    assert "simpleUrdf" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert "duck.urdf" == sphereObst.urdf()[-9:]


def test_yamlLoad():
    yamlFile = os.path.join(os.path.dirname(__file__), 'yamlExamples/urdfSphere.yaml')
    sphereObst = UrdfObstacle(fileName=yamlFile)
    assert "simpleUrdf" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert "sphere_015.urdf" in sphereObst.urdf()
