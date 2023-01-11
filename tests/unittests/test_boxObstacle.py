from omegaconf.errors import MissingMandatoryValue
import pytest
import numpy as np

from MotionPlanningEnv.boxObstacle import BoxObstacle
from MotionPlanningEnv.boxObstacle import BoxObstacleMissmatchDimensionError
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError


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

def test_boxObstacle():
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
    assert isinstance(box_obst.evaluate(t=0), list)
    assert isinstance(box_obst.evaluate(t=0)[0], np.ndarray)
    assert "simpleBox" == box_obst.name()
    assert -0.1 == box_obst.position()[0]
    assert 0.1 == box_obst.position()[1]
    assert 0.2 == box_obst.position()[2]
    assert 0.2 == box_obst.height()
    assert 3 == box_obst.dimension()


def test_errorRaiseIncompleteDict():
    obst_dict= {'type': 'box', 'geometry': {'position': [0.1, 0.2]}}
    with pytest.raises(MissingMandatoryValue):
        box_obst= BoxObstacle(name='simpleBox', content_dict=obst_dict)
        box_obst.length()

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

