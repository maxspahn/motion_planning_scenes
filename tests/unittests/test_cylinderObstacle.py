import pytest

from MotionPlanningEnv.cylinderObstacle import CylinderObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError

def test_cylinderObstacle():
    obstDict = {'type': 'cylinder', 'position': [0.1, 0.2, 0.4], 'geometry': {'radius': 0.5, 'height': 0.5}}
    sphereObst = CylinderObstacle(name='simpleCylinder', content_dict=obstDict)
    assert "simpleCylinder" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert 0.5 == sphereObst.radius()
    assert 0.5 == sphereObst.height()


def test_errorRaiseIncompleteDict():
    obstDict = {'type': 'cylinder', 'position': [0.1, 0.2], 'geometry': {'radius': 0.9}}
    with pytest.raises(ComponentIncompleteError):
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_radius_height_incorrect_type():
    notFloats = [[0.5], [0.5, 0,5], None, {}]
    
    for nofloat in notFloats:

        obstDict = {'type': 'cylinder', 'position': [0.1, 0.2, 0.5], 'geometry': {'radius': nofloat, 'height': 0.5}}
        with pytest.raises(ValueError) as _:
            CylinderObstacle(name='simpleCylinder', content_dict=obstDict)
        obstDict = {'type': 'cylinder', 'position': [0.1, 0.2, 0.5], 'geometry': {'radius': 0.5, 'height': nofloat}}
        with pytest.raises(ValueError) as _:
                CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_orientation_incorrect_type():
    obstDict = {'type': 'cylinder', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'radius': 0.5, 'height': 0.5},
            'orientation': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_orientation_incorrect_shape():
    obstDict = {'type': 'box', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'radius': 0.5, 'height': 0.5},
            'orientation': [1, 0.4, 0.8, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_color_incorrect_shape():
    obstDict = {'type': 'cylinder', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'radius': 0.5, 'height': 0.5},
            'color': [1, 0, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_negative_mass():
    obstDict = {'type': 'cylinder', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'radius': 0.5, 'height': 0.5},
            'mass': -15, 
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_color_incorrect_type():
    obstDict = {'type': 'cylinder', 
            'position': [0.1, 0.2, 0.5], 
            'geometry': {'radius': 0.5, 'height': 0.5},
            'color': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError):
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

