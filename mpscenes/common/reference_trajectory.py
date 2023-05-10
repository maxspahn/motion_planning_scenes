from abc import ABC, abstractmethod
import sympy as sp

from mpscenes.common.errors import TrajectoryComponentMissingError

t = sp.symbols("t")


class ReferenceTrajectory(ABC):
    def __init__(self, n: int, **kwargs):
        assert isinstance(n, int)
        self._n = n
        self._config = kwargs

    def n(self):
        return self._n

    def trajectory_dictionary(self) -> dict:
        if "traj" not in self._config:
            raise TrajectoryComponentMissingError(
                "Reference Trajectory definition not complete. Missing component: traj"
            )
        return self._config["traj"]

    @abstractmethod
    def concretize(self):
        pass

    @abstractmethod
    def evaluate(self, time_step: float):
        pass
