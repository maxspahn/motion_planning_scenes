import pytest

from MotionPlanningEnv.boxObstacle import BoxObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError

def test_rectangleObstacle():
    # TODO: implement when boxObstacle has 2D compatability
    pass
    # obstDict = {'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    # sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    # assert "simpleSphere" == sphereObst.name()
    # assert [0.1, 0.2] == sphereObst.position()
    # assert 0.2 == sphereObst.radius()

def test_boxObstacle():
    obstDict = {'type': 'box', 'position': [0.1, 0.2, 0.4], 'geometry': {'length': 0.5, 'width': 0.5, 'heigth': 0.5}}
    boxObst= BoxObstacle(name='simpleBox', content_dict=obstDict)
    assert "simpleBox" == boxObst.name()
    assert [0.1, 0.2, 0.4] == boxObst.position()
    assert 0.5 == boxObst.length()
    assert 0.5 == boxObst.width()
    assert 0.5 == boxObst.heigth()


def test_errorRaiseIncompleteDict():
    obstDict = {'type': 'box', 'position': [0.1, 0.2], 'geometry': {'length': 0.9}}
    with pytest.raises(ComponentIncompleteError):
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_length_width_heigth_incorrect_type():
    notFloats = [[0.5], [0.5, 0,5], None, {}]
    
    for nofloat in notFloats:

        obstDict = {'type': 'box', 'position': [0.1, 0.2, 0.5], 'geometry': {'length': nofloat, 'width': 0.5, 'heigth': 0.5}}
        with pytest.raises(ValueError) as _:
            BoxObstacle(name='simpleBox', content_dict=obstDict)
        obstDict = {'type': 'box', 'position': [0.1, 0.2, 0.5], 'geometry': {'length': 0.5, 'width': nofloat, 'heigth': 0.5}}
        with pytest.raises(ValueError) as _:
                BoxObstacle(name='simpleBox', content_dict=obstDict)
        obstDict = {'type': 'box', 'position': [0.1, 0.2, 0.5], 'geometry': {'length': 0.5, 'width': 0.5, 'heigth': nofloat}}
        with pytest.raises(ValueError) as _:
                BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_orientation_incorrect_type():
    obstDict = {'type': 'box', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'orientation': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_orientation_incorrect_shape():
    obstDict = {'type': 'box', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'orientation': [1, 0.4, 0.8, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_color_incorrect_shape():
    obstDict = {'type': 'box', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'color': [1, 0, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_negative_mass():
    obstDict = {'type': 'box', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'mass': -15, 
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)


def test_color_incorrect_type():
    obstDict = {'type': 'box', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'color': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

