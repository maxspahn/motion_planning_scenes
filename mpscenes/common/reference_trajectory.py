from abc import ABC, abstractmethod
import sympy as sp

t = sp.symbols('t')


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
