from omegaconf.errors import MissingMandatoryValue
import pytest
import numpy as np

from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.common.errors import MissmatchDimensionError, ComponentIncompleteError


def test_circleObstacle():
    obst_dict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    assert "simpleSphere" == sphere_obst.name()
    assert isinstance(sphere_obst.position(), np.ndarray)
    assert [0.1, 0.2] == sphere_obst.position().tolist()
    assert 0.2 == sphere_obst.radius()
    assert 2 == sphere_obst.dimension()

def test_sphereObstacle():
    obst_dict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    assert isinstance(sphere_obst.evaluate(t=0), list)
    assert isinstance(sphere_obst.evaluate(t=0)[0], np.ndarray)
    assert "simpleSphere" == sphere_obst.name()
    assert 0.1 == sphere_obst.position()[0]
    assert 0.2 == sphere_obst.position()[1]
    assert 0.4 == sphere_obst.position()[2]
    assert 0.2 == sphere_obst.radius()
    assert 3 == sphere_obst.dimension()


def test_errorRaiseIncompleteDict():
    obst_dict= {'type': 'sphere', 'geometry': {'position': [0.1, 0.2]}}
    with pytest.raises(MissingMandatoryValue):
        sphere_obst= SphereObstacle(name='simpleSphere', content_dict=obst_dict)
        sphere_obst.radius()

def test_mask_selection():
    obst_dict= {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    mask = ["type", "position", "radius"]
    selected_items = sphere_obst.evaluate_components(mask, 0)
    assert isinstance(selected_items, dict)
    assert selected_items['type'] == 'sphere'
    assert selected_items['position'][0] == 0.1
    assert selected_items['position'][1] == 0.2
    assert selected_items['position'][2] == 0.4
    assert list(selected_items.keys()) == ["type", "position", "radius"]


def test_saving_obstacle():
    obst_dict= {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    sphere_obst.shuffle()
    obst_dict_after = sphere_obst.dict()
    assert isinstance(obst_dict_after, dict)
    assert obst_dict_after['geometry']['position'][0] != 0.1
    assert isinstance(obst_dict_after['geometry']['position'], list)

