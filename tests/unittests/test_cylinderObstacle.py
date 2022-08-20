import pytest

from MotionPlanningEnv.cylinderObstacle import CylinderObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError
from MotionPlanningEnv.cylinderObstacle import CylinderObstacleMissmatchDimensionError

def test_cylinderObstacle():
    obstDict = {'dimension': 3, 'type': 'cylinder', 'geometry': {'position': [0.1, 0.2, 0.4], 'radius': 0.5, 'heigth': 0.5}}
    sphereObst = CylinderObstacle(name='simpleCylinder', content_dict=obstDict)
    assert "simpleCylinder" == sphereObst.name()
    assert [0.1, 0.2, 0.4] == sphereObst.position()
    assert 0.5 == sphereObst.radius()
    assert 0.5 == sphereObst.heigth()


def test_errorRaiseIncompleteDict():
    obstDict = {'dimension': 2, 'type': 'cylinder', 'geometry': {'position': [0.1, 0.2], 'radius': 0.9}}
    with pytest.raises(ComponentIncompleteError):
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)


def test_errorRaiseMissmatichDimension():
    obstDict = {'dimension': 3, 'type': 'box', 'geometry': {'position': [0.1, 0.2], 'radius': 0.5, 'heigth': 0.5}}
    with pytest.raises(CylinderObstacleMissmatchDimensionError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_radius_heigth_incorrect_type():
    notFloats = [[0.5], [0.5, 0,5], None, {}]
    
    for nofloat in notFloats:

        obstDict = {'dimension': 3, 'type': 'cylinder', 'geometry': {'position': [0.1, 0.2, 0.5], 'radius': nofloat, 'heigth': 0.5}}
        with pytest.raises(ValueError) as _:
            CylinderObstacle(name='simpleCylinder', content_dict=obstDict)
        obstDict = {'dimension': 3, 'type': 'cylinder', 'geometry': {'position': [0.1, 0.2, 0.5], 'radius': 0.5, 'heigth': nofloat}}
        with pytest.raises(ValueError) as _:
                CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_orientation_incorrect_type():
    obstDict = {'dimension': 3,
            'type': 'cylinder', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'radius': 0.5, 'heigth': 0.5},
            'orientation': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_orientation_incorrect_shape():
    obstDict = {'dimension': 3,
            'type': 'box', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'radius': 0.5, 'heigth': 0.5},
            'orientation': [1, 0.4, 0.8, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_color_incorrect_shape():
    obstDict = {'dimension': 3,
            'type': 'cylinder', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'radius': 0.5, 'heigth': 0.5},
            'color': [1, 0, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_negative_mass():
    obstDict = {'dimension': 3,
            'type': 'cylinder', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'radius': 0.5, 'heigth': 0.5},
            'mass': -15, 
            }
    with pytest.raises(ValueError) as _:
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

def test_color_incorrect_type():
    obstDict = {'dimension': 3,
            'type': 'cylinder', 
            'geometry': {'position': [0.1, 0.2, 0.5], 'radius': 0.5, 'heigth': 0.5},
            'color': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError):
        CylinderObstacle(name='simpleCylinder', content_dict=obstDict)

