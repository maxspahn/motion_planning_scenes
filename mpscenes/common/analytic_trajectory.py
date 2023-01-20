import numpy as np
import sympy as sp

from mpscenes.common.errors import TrajectoryComponentMissingError
from mpscenes.common.reference_trajectory import ReferenceTrajectory, t

def evaluate_component(expression, evaluation_time: float):
    evaluation = []
    for expr in expression:
        if isinstance(expr, float):
            evaluation.append(expr)
        else:
            evaluation.append(float(expr.evalf(subs={'t': evaluation_time})))
    return np.array(evaluation)


class AnalyticTrajectory(ReferenceTrajectory):
    def __init__(self, n: int, **kwargs):
        super().__init__(n)
        if 'traj' not in kwargs:
            raise TrajectoryComponentMissingError("Trajectory definition not complete. Missing component: traj")
        self._t = t
        self._traj = [eval(x) for x in kwargs.get('traj')]

    def concretize(self):
        self._v = [sp.diff(component, t) for component in self._traj]
        self._a = [sp.diff(component, t) for component in self._v]

    def shuffle(self):
        pass


    def evaluate(self, t):
        x = evaluate_component(self._traj, t)
        v = evaluate_component(self._v, t)
        a = evaluate_component(self._a, t)
        return [x, v, a]
