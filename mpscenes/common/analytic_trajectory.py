import numpy as np
import sympy as sp

from mpscenes.common.reference_trajectory import ReferenceTrajectory, t


def evaluate_component(expression, evaluation_time: float):
    evaluation = []
    for expr in expression:
        if isinstance(expr, float):
            evaluation.append(expr)
        else:
            evaluation.append(float(expr.evalf(subs={"t": evaluation_time})))
    return np.array(evaluation)


class AnalyticTrajectory(ReferenceTrajectory):
    def __init__(self, n: int, **kwargs):
        super().__init__(n, **kwargs)
        self._t = t
        self._traj = [eval(x) for x in self.trajectory_dictionary()]

    def concretize(self):
        self._velocity = [sp.diff(component, t) for component in self._traj]
        self._acceleration = [sp.diff(component, t) for component in self._velocity]

    def shuffle(self):
        pass

    def evaluate(self, time_step: float) -> list:
        position = evaluate_component(self._traj, time_step)
        velocity = evaluate_component(self._velocity, time_step)
        acceleration = evaluate_component(self._acceleration, time_step)
        return [position, velocity, acceleration]
