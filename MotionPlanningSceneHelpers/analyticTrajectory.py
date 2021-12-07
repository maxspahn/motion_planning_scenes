import casadi as ca
import numpy as np

from MotionPlanningSceneHelpers.referenceTrajectory import ReferenceTrajectory, t


class TrajectoryComponentMissingError(Exception):
    pass


class AnalyticTrajectory(ReferenceTrajectory):
    def __init__(self, n: int, **kwargs):
        super().__init__(n)
        if 'traj' not in kwargs:
            raise TrajectoryComponentMissingError("Trajectory definition not complete. Missing component: traj")
        self._t = t
        self._traj = ca.vcat([eval(x) for x in kwargs.get('traj')])

    def concretize(self):
        self._v = ca.jacobian(self._traj, self._t)
        self._a = ca.jacobian(self._v, self._t)
        self._funs = ca.Function("traj", [self._t], [self._traj, self._v, self._a])

    def evaluate(self, t):
        fun = self._funs(t)
        x = np.array(fun[0])[:, 0]
        v = np.array(fun[1])[:, 0]
        a = np.array(fun[2])[:, 0]
        return [x, v, a]
