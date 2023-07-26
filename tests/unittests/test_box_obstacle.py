from omegaconf.errors import MissingMandatoryValue
import pytest
import numpy as np

from mpscenes.obstacles.box_obstacle import BoxObstacle


def test_rectangleObstacle():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[0.1, 0.2],
            'length': 0.2,
            'width': 0.2,
        }
    }
    box_obst = BoxObstacle(name='simpleRectangle', content_dict=obst_dict)
    assert "simpleRectangle" == box_obst.name()
    assert isinstance(box_obst.position(), np.ndarray)
    assert [0.2, 0.2] == [box_obst.length(), box_obst.width()]
    assert 2 == box_obst.dimension()

def test_rgba():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[0.1, 0.2],
            'length': 0.2,
            'width': 0.2,
        },
        'rgba': [0.0, 0.0, 0.0, 0.1],
    }
    box_obst = BoxObstacle(name='simpleRectangle', content_dict=obst_dict)
    assert box_obst.rgba()[0] == 0.0


def test_boxObstacle():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[-0.1, 0.1, 0.2],
            'orientation': [1.0, 0.0, 0.0, 0.0],
            'length': 0.2,
            'width': 0.2,
            'height': 0.2,
        }
    }
    box_obst = BoxObstacle(name='simpleBox', content_dict=obst_dict)
    assert isinstance(box_obst.evaluate(t=0), list)
    assert isinstance(box_obst.evaluate(t=0)[0], np.ndarray)
    assert "simpleBox" == box_obst.name()
    assert -0.1 == box_obst.position()[0]
    assert 0.1 == box_obst.position()[1]
    assert 0.2 == box_obst.position()[2]
    assert 0.2 == box_obst.height()
    assert 3 == box_obst.dimension()
    assert isinstance(box_obst.orientation(t=0), np.ndarray)


def test_errorRaiseIncompleteDict():
    obst_dict= {'type': 'box', 'geometry': {'position': [0.1, 0.2]}}
    box_obst= BoxObstacle(name='simpleBox', content_dict=obst_dict)
    assert 1.0 == box_obst.length()

def test_mask_selection():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[0.1, 0.2, 0.4],
            'length': 0.2,
            'width': 0.2,
            'height': 0.2,
        }
    }
    box_obst = BoxObstacle(name='simpleBox', content_dict=obst_dict)
    mask = ["type", "position", "length"]
    selected_items = box_obst.evaluate_components(mask, 0)
    assert isinstance(selected_items, dict)
    assert selected_items['type'] == 'box'
    assert selected_items['position'][0] == 0.1
    assert selected_items['position'][1] == 0.2
    assert selected_items['position'][2] == 0.4
    assert list(selected_items.keys()) == ["type", "position", "length"]


def test_saving_obstacle():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[-0.1, 0.1, 0.2],
            'length': 0.2,
            'width': 0.2,
            'height': 0.2,
        }
    }
    box_obst = BoxObstacle(name='simpleBox', content_dict=obst_dict)
    box_obst.shuffle()
    obst_dict_after = box_obst.dict()
    assert isinstance(obst_dict_after, dict)
    assert obst_dict_after['geometry']['position'][0] != -0.1
    assert isinstance(obst_dict_after['geometry']['position'], list)


def test_distance():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[-0.1, 0.5, 0.7],
            'length': 0.3,
            'width': 0.7,
            'height': 0.8,
        }
    }
    '''
    -0.25 < x < 0.05
    0.15 < y < 0.85
    0.3 < z < 1.1
    '''
    box_obst = BoxObstacle(name='simpleBox', content_dict=obst_dict)
    point = np.array([0.5, 0.8, 0.4])
    '''
    dx = 0.45
    dy = 0
    dz = 0
    '''
    distance = box_obst.distance(point)
    assert isinstance(distance, float)
    distance_truth = 0.45
    assert distance == pytest.approx(distance_truth)
    point = np.array([0.5, 1.8, 0.4])
    '''
    dx = 0.45
    dy = 0.95
    dz = 0
    '''
    distance = box_obst.distance(point)
    assert isinstance(distance, float)
    distance_truth = np.sqrt(0.45**2+0.95**2)
    assert distance == pytest.approx(distance_truth)
    point = np.array([0.5, 1.8, -0.2])
    '''
    dx = 0.45
    dy = 0.95
    dz = 0.5
    '''
    distance = box_obst.distance(point)
    assert isinstance(distance, float)
    distance_truth = np.sqrt(0.45**2+0.95**2+0.5**2)
    assert distance == pytest.approx(distance_truth)


def test_multi_distance():
    obst_dict = {
        'type': 'box',
        'geometry': {
            'position':[-0.1, 0.5, 0.7],
            'length': 0.3,
            'width': 0.7,
            'height': 0.8,
        }
    }
    box_obst = BoxObstacle(name='simpleBox', content_dict=obst_dict)
    points = np.array([[0.5, 0.8, 0.4], [0.5, 1.8, -0.2]])
    distances = box_obst.distance(points)
    print(distances)
    assert isinstance(distances, np.ndarray)
    distances_truth = np.array([0.45, np.sqrt(0.45**2+0.95**2+0.5**2)])
    assert distances[0] == pytest.approx(distances_truth[0])
    assert distances[1] == pytest.approx(distances_truth[1])

