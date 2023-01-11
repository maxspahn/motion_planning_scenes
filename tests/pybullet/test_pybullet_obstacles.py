import pytest
import time
import os
import sys

import pybullet as p
import pybullet_data

from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.dynamic_box_obstacle import DynamicBoxObstacle
from mpscenes.obstacles.dynamic_urdf_obstacle import DynamicUrdfObstacle
from mpscenes.obstacles.dynamic_cylinder_obstacle import DynamicCylinderObstacle

gui = False

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

def test_sphere_obstacle(bullet):
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    body_id = sphereObst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        sphereObst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

def test_box_obstacle_static(bullet):
    obstDict = {'type': 'box', 'movable': True, 'geometry': {'position': [0.1, 0.2, 0.4], 'length': 2.2, 'width': 0.3, 'height': 0.2}}
    boxObst = BoxObstacle(name='simpleBox', content_dict=obstDict)
    body_id = boxObst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        boxObst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

def test_cylinder_obstacle_static(bullet):
    obst_dict = {'type': 'cylinder', 'movable': True, 'geometry': {'position': [0.0, 0.0, 1.4], 'radius': 1.0, 'height': 1.0}}
    cylinder_obst = CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    body_id = cylinder_obst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        cylinder_obst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_cylinder_obstacle_static_gui(bullet_gui):
    obst_dict = {'type': 'cylinder', 'movable': True, 'geometry': {'position': [0.0, 0.0, 1.0], 'radius': 1.0, 'height': 1.0}}
    cylinder_obst = CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    body_id = cylinder_obst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        cylinder_obst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()


def test_two_obstacles(bullet):
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    body_id = sphereObst.add_to_bullet(bullet)
    obstDict = {'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.4], 'length': 2.2, 'width': 0.3, 'height': 0.2}}
    boxObst = BoxObstacle(name='simpleBox', content_dict=obstDict)
    body_id_2 = boxObst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    assert isinstance(body_id_2, int)
    for i in range(100):
        bullet.stepSimulation()
    bullet.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_sphere_obstacle_gui(bullet_gui):
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    body_id = sphereObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_box_obstacle_static_gui(bullet_gui):
    obstDict = {'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.4], 'length': 2.2, 'width': 0.3, 'height': 0.2}}
    boxObst = BoxObstacle(name='simpleBox', content_dict=obstDict)
    body_id = boxObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        boxObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_two_obstacles_gui(bullet_gui):
    obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    body_id = sphereObst.add_to_bullet(bullet_gui)
    obstDict = {'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.4], 'length': 2.2, 'width': 0.3, 'height': 0.2}}
    boxObst = BoxObstacle(name='simpleBox', content_dict=obstDict)
    body_id_2 = boxObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    assert isinstance(body_id_2, int)
    for _ in range(100):
        bullet_gui.stepSimulation()
        time.sleep(1/100)
    bullet_gui.disconnect()


def test_urdf_obstacle_static(bullet):
    obstDict = {
        "type": "sphere",
        "geometry": {"position": [0.1, 0.2, 0.4]},
        "urdf": "teddy_large.urdf",
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", content_dict=obstDict)
    body_id = sphereObst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        sphereObst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_urdf_obstacle_gui(bullet_gui):
    obstDict = {
        "type": "sphere",
        "geometry": {"position": [0.1, 0.2, 0.4]},
        "urdf": "teddy_large.urdf",
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", content_dict=obstDict)
    body_id = sphereObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_circle_obstacle_gui(bullet_gui):
    obstDict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    body_id = dynamicSphereObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_circle_obstacle(bullet):
    obstDict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    body_id = dynamicSphereObst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

def test_spline_obstacle(bullet):
    splineDict = {'degree': 2, 'controlPoints': [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], 'duration': 10}
    obstDict = {
        "type": "splineSphere",
        "geometry": {"trajectory": splineDict, "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    body_id = dynamicSphereObst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_spline_obstacle_gui(bullet_gui):
    splineDict = {'degree': 2, 'controlPoints': [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], 'duration': 10}
    obstDict = {
        "type": "splineSphere",
        "geometry": {"trajectory": splineDict, "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obstDict
    )
    body_id = dynamicSphereObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_box_obstacle_gui(bullet_gui):
    obstDict = {
        "type": "box",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "width": 0.5, 'length': 0.2, 'height': 0.01},
    }
    dynamicBoxObst = DynamicBoxObstacle(
        name="dynamicbox", content_dict=obstDict
    )
    body_id = dynamicBoxObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicBoxObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_box_obstacle(bullet):
    obstDict = {
        "type": "box",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "width": 0.5, 'length': 0.2, 'height': 0.01},
    }
    dynamic_box_obst= DynamicBoxObstacle(
        name="dynamicbox", content_dict=obstDict
    )
    body_id = dynamic_box_obst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        dynamic_box_obst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_urdf_obstacle_gui(bullet_gui):
    obstDict = {
        "type": "urdf",
        "geometry": {"trajectory": ["0.01 * t - 2", "0.02 * t - 1", "0.5"]},
        "urdf": "teddy_large.urdf",
        "scaling": 0.1
    }
    dynamicUrdfObst = DynamicUrdfObstacle(
        name="dynamicurdf", content_dict=obstDict
    )
    body_id = dynamicUrdfObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicUrdfObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_urdf_obstacle(bullet):
    obstDict = {
        "type": "urdf",
        "geometry": {"trajectory": ["0.1 * t", "-0.2 * t ", "0.5"]},
        "urdf": "teddy_large.urdf",
    }
    dynamic_urdf_obst= DynamicUrdfObstacle(
        name="dynamicurdf", content_dict=obstDict
    )
    body_id = dynamic_urdf_obst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        dynamic_urdf_obst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(not gui, reason="Not testing because gui is not available")
def test_cylinder_obstacle_gui(bullet_gui):
    obstDict = {
        "type": "cylinder",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t", "1.0"], "radius": 0.5, 'height': 0.1},
    }
    dynamicCylinderObst = DynamicCylinderObstacle(
        name="dynamiccylinder", content_dict=obstDict
    )
    body_id = dynamicCylinderObst.add_to_bullet(bullet_gui)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicCylinderObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_cylinder_obstacle(bullet):
    obstDict = {
        "type": "cylinder",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.5, 'height': 0.01},
    }
    dynamic_cylinder_obst= DynamicCylinderObstacle(
        name="dynamiccylinder", content_dict=obstDict
    )
    body_id = dynamic_cylinder_obst.add_to_bullet(bullet)
    assert isinstance(body_id, int)
    for i in range(100):
        bullet.stepSimulation()
        dynamic_cylinder_obst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()
