from omegaconf.errors import MissingMandatoryValue
import pytest
import numpy as np

from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle
from mpscenes.common.errors import MissmatchDimensionError, ComponentIncompleteError


def test_cylinder_obstacle():
    obst_dict = {
        'type': 'cylinder',
        'geometry': {
            'position':[0.1, 0.2],
            'radius': 0.2,
            'height': 0.2,
        }
    }
    cylinder_obst = CylinderObstacle(name='simpleRectangle', content_dict=obst_dict)
    assert "simpleRectangle" == cylinder_obst.name()
    assert isinstance(cylinder_obst.position(), np.ndarray)
    assert [0.2, 0.2] == [cylinder_obst.radius(), cylinder_obst.height()]
    assert 2 == cylinder_obst.dimension()

def test_cylinderObstacle():
    obst_dict = {
        'type': 'cylinder',
        'geometry': {
            'position':[-0.1, 0.1, 0.2],
            'radius': 0.1,
            'height': 0.2,
        }
    }
    cylinder_obst = CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    assert isinstance(cylinder_obst.evaluate(t=0), list)
    assert isinstance(cylinder_obst.evaluate(t=0)[0], np.ndarray)
    assert "simpleCylinder" == cylinder_obst.name()
    assert -0.1 == cylinder_obst.position()[0]
    assert 0.1 == cylinder_obst.position()[1]
    assert 0.2 == cylinder_obst.position()[2]
    assert 0.2 == cylinder_obst.height()
    assert 3 == cylinder_obst.dimension()


def test_errorRaiseIncompleteDict():
    obst_dict= {'type': 'cylinder', 'geometry': {'position': [0.1, 0.2]}}
    cylinder_obst= CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    assert 1.0 == cylinder_obst.height()

def test_mask_selection():
    obst_dict = {
        'type': 'cylinder',
        'geometry': {
            'position':[0.1, 0.2, 0.4],
            'radius': 0.2,
            'height': 0.2,
        }
    }
    cylinder_obst = CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    mask = ["type", "position", "radius"]
    selected_items = cylinder_obst.evaluate_components(mask, 0)
    assert isinstance(selected_items, dict)
    assert selected_items['type'] == 'cylinder'
    assert selected_items['position'][0] == 0.1
    assert selected_items['position'][1] == 0.2
    assert selected_items['position'][2] == 0.4
    assert list(selected_items.keys()) == ["type", "position", "radius"]


def test_saving_obstacle():
    obst_dict = {
        'type': 'cylinder',
        'geometry': {
            'position':[-0.1, 0.1, 0.2],
            'radius': 0.1,
            'height': 0.2,
        }
    }
    cylinder_obst = CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    cylinder_obst.shuffle()
    obst_dict_after = cylinder_obst.dict()
    assert isinstance(obst_dict_after, dict)
    assert obst_dict_after['geometry']['position'][0] != -0.1
    assert isinstance(obst_dict_after['geometry']['position'], list)

def test_distance():
    obst_dict = {
        'type': 'cylinder',
        'geometry': {
            'position':[-0.1, 0.1, 0.2],
            'radius': 0.1,
            'height': 0.2,
        }
    }
    '''
    -0.2 < x < 0.0
    0.0 < y < 0.2
    0.1 < z < 0.3
    '''
    cylinder_obst = CylinderObstacle(name='simpleCylinder', content_dict=obst_dict)
    point = np.array([0.5, 0.8, 0.4])
    '''
    dr = sqrt(0.6^2 + 0.7^2) - 0.1
    dz = 0.1
    '''
    distance = cylinder_obst.distance(point)
    assert isinstance(distance, float)
    distance_truth = np.sqrt(0.1**2 + (np.sqrt(0.6**2 + 0.7**2) - 0.1)**2)
    assert distance == pytest.approx(distance_truth)


