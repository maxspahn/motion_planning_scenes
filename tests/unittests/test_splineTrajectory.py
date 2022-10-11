import numpy as np
import pytest

from MotionPlanningSceneHelpers.splineTrajectory import (
    SplineTrajectory,
    TrajectoryComponentMissingError,
)


@pytest.fixture
def simpleSplineTrajectory():
    traj = SplineTrajectory(
        2,
        traj={
            "degree": 2,
            "controlPoints": [[1.0, 0.0], [2.0, 0.0], [2.0, 1.0]],
            "duration": 10,
            "high": {
                "controlPoints": [[2.0, 1.0], [2.0, 1.0], [3.0, 0.2]],
            },
            "low": {
                "controlPoints": [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            }
        },
    )
    traj.concretize()
    return traj


def test_splineTrajectory(simpleSplineTrajectory):
    x, v, a = simpleSplineTrajectory.evaluate(0.3)
    assert isinstance(x, np.ndarray)
    assert isinstance(v, np.ndarray)
    assert isinstance(a, np.ndarray)
    assert x[0] == pytest.approx(1.00443311e-0, abs=1e-5)
    assert x[1] == pytest.approx(4.92403954e-6, abs=1e-5)
    v_norm = np.linalg.norm(v)
    v_norm_verification = np.pi / 10 * np.sin(0.3 * np.pi / 10)
    assert v_norm == pytest.approx(v_norm_verification)
    a_norm = np.linalg.norm(a)
    a_norm_verification = np.pi / 10 * np.cos(0.3 * np.pi / 10)
    assert a_norm == pytest.approx(a_norm_verification)


def test_raiseComponentMissingError():
    with pytest.raises(TrajectoryComponentMissingError):
        SplineTrajectory(3)


def test_shuffle_trajectory(simpleSplineTrajectory):
    simpleSplineTrajectory.shuffle()
    x, v, a = simpleSplineTrajectory.evaluate(0.3)
    assert isinstance(x, np.ndarray)
    assert isinstance(v, np.ndarray)
    assert isinstance(a, np.ndarray)
    assert x[0] != pytest.approx(1.00443311e-0, abs=1e-5)
    assert x[1] != pytest.approx(4.92403954e-6, abs=1e-5)
