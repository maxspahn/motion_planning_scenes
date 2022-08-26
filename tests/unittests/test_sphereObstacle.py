from MotionPlanningEnv.sphereObstacle import SphereObstacle

def test_circle_obstacle():
    obst_dict = {"type": "sphere", "position": [0.1, 0.2],
            "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simpleSphere", content_dict=obst_dict)
    assert "simpleSphere" == sphere_obst.name()
    assert [0.1, 0.2] == sphere_obst.position()
    assert 0.2 == sphere_obst.radius()
    assert 2 == sphere_obst.dimension()

def test_sphere_obstacle():
    obst_dict = {"type": "sphere", "position": [0.1, 0.2, 0.4],
            "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simpleSphere",
            content_dict=obst_dict)
    assert "simpleSphere" == sphere_obst.name()
    assert [0.1, 0.2, 0.4] == sphere_obst.position()
    assert 0.2 == sphere_obst.radius()
    assert 3 == sphere_obst.dimension()

def test_saving_obstacle():
    obst_dict = {"type": "sphere", "position": [0.1, 0.2, 0.4],
            "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simpleSphere", content_dict=obst_dict)
    sphere_obst.shuffle()
    obst_dict_after = sphere_obst.dict()
    assert isinstance(obst_dict_after, dict)
    assert obst_dict_after["position"][0] != 0.1
