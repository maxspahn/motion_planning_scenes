import pytest

from MotionPlanningEnv.boxObstacle import BoxObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError
from MotionPlanningEnv.boxObstacle import BoxObstacleMissmatchDimensionError

def test_rectangleObstacle():
    # TODO: implement when boxObstacle has 2D compatability
    pass
    # obstDict = {'dimension': 2, 'type': 'sphere', 'geometry': {'position': [0.1, 0.2], 'radius': 0.2}}
    # sphereObst = SphereObstacle(name='simpleSphere', content_dict=obstDict)
    # assert "simpleSphere" == sphereObst.name()
    # assert [0.1, 0.2] == sphereObst.position()
    # assert 0.2 == sphereObst.radius()

def test_boxObstacle():
    obstDict = {'dimension': 3, 'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.4], 'length': 0.5, 'width': 0.5, 'heigth': 0.5}}
    sphereObst = BoxObstacle(name='simpleBox', content_dict=obstDict)
    assert "simpleBox" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert 0.5 == sphereObst.length()
    assert 0.5 == sphereObst.width()
    assert 0.5 == sphereObst.heigth()


def test_errorRaiseIncompleteDict():
    obstDict = {'dimension': 2, 'type': 'box', 'geometry': {'position': [0.1, 0.2], 'length': 0.9}}
    with pytest.raises(ComponentIncompleteError):
        BoxObstacle(name='simpleBox', content_dict=obstDict)


def test_errorRaiseMissmatichDimension():
    obstDict = {'dimension': 3, 'type': 'box', 'geometry': {'position': [0.1, 0.2], 'length': 0.5, 'width': 0.5, 'heigth': 0.5}}
    with pytest.raises(BoxObstacleMissmatchDimensionError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_length_width_heigth_incorrect_type():
    notFloats = [[0.5], [0.5, 0,5], None, {}]
    
    for nofloat in notFloats:

        obstDict = {'dimension': 3, 'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.5], 'length': nofloat, 'width': 0.5, 'heigth': 0.5}}
        with pytest.raises(ValueError) as _:
            BoxObstacle(name='simpleBox', content_dict=obstDict)
        obstDict = {'dimension': 3, 'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': nofloat, 'heigth': 0.5}}
        with pytest.raises(ValueError) as _:
                BoxObstacle(name='simpleBox', content_dict=obstDict)
        obstDict = {'dimension': 3, 'type': 'box', 'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': 0.5, 'heigth': nofloat}}
        with pytest.raises(ValueError) as _:
                BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_orientation_incorrect_type():
    obstDict = {'dimension': 3,
            'type': 'box', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'orientation': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_orientation_incorrect_shape():
    obstDict = {'dimension': 3,
            'type': 'box', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'orientation': [1, 0.4, 0.8, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_color_incorrect_shape():
    obstDict = {'dimension': 3,
            'type': 'box', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'color': [1, 0, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)

def test_negative_mass():
    obstDict = {'dimension': 3,
            'type': 'box', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'mass': -15, 
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)


def test_color_incorrect_type():
    obstDict = {'dimension': 3,
            'type': 'box', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'length': 0.5, 'width': 0.5, 'heigth': 0.5},
            'color': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obstDict)
