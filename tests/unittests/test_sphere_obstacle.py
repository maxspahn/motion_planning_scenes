import pytest
import numpy as np

from mpscenes.obstacles.sphere_obstacle import SphereObstacle


def test_circleObstacle():
    obst_dict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    assert "simpleSphere" == sphere_obst.name()
    assert isinstance(sphere_obst.position(), np.ndarray)
    assert isinstance(sphere_obst.orientation(), np.ndarray)
    assert sphere_obst.orientation().size == 4
    assert [0.1, 0.2] == sphere_obst.position().tolist()
    assert 0.2 == sphere_obst.radius()
    assert 2 == sphere_obst.dimension()
    assert 0.0 == sphere_obst.rgba()[0]

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
    sphere_obst= SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    assert 1.0 == sphere_obst.radius()

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

def test_distance():
    obst_dict= {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    point = np.array([1.0, 0.2, 0.1])
    distance = sphere_obst.distance(point)
    assert isinstance(distance, float)
    distance_truth = np.linalg.norm(np.array([0.1, 0.2, 0.4]) - point) - 0.2
    assert distance == pytest.approx(distance_truth)

def test_multi_distance():
    obst_dict= {'type': 'sphere', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.2}}
    sphere_obst = SphereObstacle(name='simpleSphere', content_dict=obst_dict)
    points = np.array([[1.0, 0.2, 0.1], [1.0, 0.2, 0.1]])
    distance = sphere_obst.distance(points)
    assert isinstance(distance, np.ndarray)
    assert isinstance(distance[0], float)
    assert isinstance(distance[1], float)
    distance_truth = np.linalg.norm(np.array([0.1, 0.2, 0.4]) - points[0]) - 0.2
    assert distance[0] == pytest.approx(distance_truth)
    assert distance[1] == pytest.approx(distance_truth)


