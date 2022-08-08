import pytest
import time
import os
import sys

import pybullet as p
import pybullet_data
from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.urdfObstacle import UrdfObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

no_gui = True

@pytest.fixture
def bullet():
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    #p.setAdditionalSearchPath(os.path.dirname(__file__) + "/models")
    return p

@pytest.fixture
def bullet_gui():
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    #p.setAdditionalSearchPath(os.path.dirname(__file__) + "/models")
    return p

def test_sphereObstacle(bullet):
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    sphereObst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        sphereObst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_sphereObstacle_gui(bullet_gui):
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    sphereObst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_urdfObstacle(bullet):
    obstDict = {
        "type": "sphere",
        "geometry": {"position": [0.1, 0.2, 0.4]},
        "urdf": "teddy_large.urdf",
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", content_dict=obstDict)
    sphereObst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        sphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_urdfObstacle_gui(bullet_gui):
    obstDict = {
        "type": "sphere",
        "geometry": {"position": [0.1, 0.2, 0.4]},
        "urdf": "teddy_large.urdf",
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", content_dict=obstDict)
    sphereObst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_circleObstacle_gui(bullet_gui):
    obstDict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    dynamicSphereObst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_circleObstacle(bullet):
    obstDict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    dynamicSphereObst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

def test_splineObstacle(bullet):
    splineDict = {'degree': 2, 'controlPoints': [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], 'duration': 10}
    obstDict = {
        "type": "splineSphere",
        "geometry": {"trajectory": splineDict, "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    dynamicSphereObst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_splineObstacle_gui(bullet_gui):
    splineDict = {'degree': 2, 'controlPoints': [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], 'duration': 10}
    obstDict = {
        "type": "splineSphere",
        "geometry": {"trajectory": splineDict, "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    dynamicSphereObst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()
