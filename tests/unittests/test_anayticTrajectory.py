import numpy as np
import pytest

from MotionPlanningSceneHelpers.analyticTrajectory import (
    AnalyticTrajectory,
    TrajectoryComponentMissingError,
)


@pytest.fixture
def simpleAnalyticTrajectory():
    traj = AnalyticTrajectory(2, traj=["0.2 * t", "0.3 * t"])
    traj.concretize()
    return traj


def test_analyticTrajectory(simpleAnalyticTrajectory):
    x, v, a = simpleAnalyticTrajectory.evaluate(0.3)
    assert isinstance(x, np.ndarray)
    assert isinstance(v, np.ndarray)
    assert isinstance(a, np.ndarray)
    assert x[0] == 0.2 * 0.3
    assert x[1] == 0.3 * 0.3
    assert v[0] == 0.2
    assert v[1] == 0.3
    assert a[0] == 0.0
    assert a[1] == 0.0


def test_raiseComponentMissingError():
    with pytest.raises(TrajectoryComponentMissingError):
        AnalyticTrajectory(3)
