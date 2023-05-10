import numpy as np

from geomdl import BSpline
from geomdl import utilities
import logging

from mpscenes.common.errors import TrajectoryComponentMissingError
from mpscenes.common.reference_trajectory import ReferenceTrajectory


class SplineTrajectory(ReferenceTrajectory):
    def __init__(self, n: int, **kwargs):
        super().__init__(n)
        self._config = kwargs
        if "degree" not in self.trajectory_dictionary().keys():
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component in trajectory: degree"
            )
        if "controlPoints" not in self.trajectory_dictionary().keys():
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component in trajectory: controlPoints"
            )
        if "duration" not in self.trajectory_dictionary().keys():
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component in trajectory: duration"
            )
        self.initialize_spline()



    def initialize_spline(self):
        self._traj = BSpline.Curve()
        self._traj.degree = self.trajectory_dictionary()["degree"]
        list_ctrlpts = [list(val) for val in self.trajectory_dictionary()["controlPoints"]]
        self._traj.ctrlpts = list_ctrlpts
        self._traj.knotvector = utilities.generate_knot_vector(
            self._traj.degree, len(self._traj.ctrlpts)
        )
        self._duration = self.trajectory_dictionary()["duration"]

    def concretize(self):
        pass

    def shuffle(self):
        limit_low = np.array(self.trajectory_dictionary()["low"]["controlPoints"])
        limit_high = np.array(self.trajectory_dictionary()["high"]["controlPoints"])
        self._config['traj']["controlPoints"] = np.random.uniform(
            limit_low, limit_high, limit_low.shape
        ).tolist()
        self.initialize_spline()

    def timeReparameterize(self, time_step: float) -> float:
        if time_step > self._duration:
            return 1.0
        return 0.5 * (1 - np.cos(np.pi * time_step / self._duration))

    def getScaledDerivatives(self, time_step: float) -> list:
        t_ref = self.timeReparameterize(time_step)
        xds = self._traj.derivatives(t_ref, order=2)
        position = np.array(xds[0])
        v_raw = np.array(xds[1])
        a_raw = np.array(xds[2])
        v_scaling = (
            1 * np.pi / self._duration * np.sin(time_step * np.pi / self._duration)
        )
        a_scaling = (
            1 * np.pi / self._duration * np.cos(time_step * np.pi / self._duration)
        )
        velocity = v_scaling * v_raw / np.linalg.norm(v_raw)
        if np.linalg.norm(a_raw) < 1e-5:
            logging.warn(
                f"Assuming zero acceleration on spline, because of large magnitude {np.linalg.norm(a_raw)}"
            )
            acceleration = a_raw * 0
        else:
            acceleration = a_scaling * a_raw / np.linalg.norm(a_raw)
        if t_ref == 1.0:
            velocity = v_raw * 0.0
            acceleration = a_raw * 0.0
        return [position, velocity, acceleration]

    def evaluate(self, time_step: float) -> list:
        return self.getScaledDerivatives(time_step)
