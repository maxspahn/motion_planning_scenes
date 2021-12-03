from abc import ABC, abstractmethod
import casadi as ca

t = ca.SX.sym("t", 1)


class ReferenceTrajectory(ABC):
    def __init__(self, n: int, **kwargs):
        assert isinstance(n, int)
        self._n = n

    def n(self):
        return self._n

    @abstractmethod
    def concretize(self):
        pass

    @abstractmethod
    def evaluate(self, t):
        pass
