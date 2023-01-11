import numpy as np

from geomdl import BSpline
from geomdl import utilities
import logging

from mpscenes.common.reference_trajectory import ReferenceTrajectory
from mpscenes.common.errors import TrajectoryComponentMissingError



class SplineTrajectory(ReferenceTrajectory):
    def __init__(self, n: int, **kwargs):
        super().__init__(n)
        if "traj" not in kwargs:
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component: traj"
            )
        self._trajDict = kwargs.get("traj")
        if "degree" not in self._trajDict.keys():
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component in trajectory: degree"
            )
        if "controlPoints" not in self._trajDict.keys():
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component in trajectory: controlPoints"
            )
        if "duration" not in self._trajDict.keys():
            raise TrajectoryComponentMissingError(
                "Spline definition not complete. Missing component in trajectory: duration"
            )
        self.initialize_spline()

    def initialize_spline(self):
        self._traj = BSpline.Curve()
        self._traj.degree = self._trajDict["degree"]
        list_ctrlpts = [list(val) for val in self._trajDict["controlPoints"]]
        self._traj.ctrlpts = list_ctrlpts
        self._traj.knotvector = utilities.generate_knot_vector(
            self._traj.degree, len(self._traj.ctrlpts)
        )
        self._duration = self._trajDict["duration"]

    def concretize(self):
        pass

    def shuffle(self):
        limit_low = np.array(self._trajDict['low']['controlPoints'])
        limit_high = np.array(self._trajDict['high']['controlPoints'])
        self._trajDict['controlPoints'] = np.random.uniform(limit_low, limit_high, limit_low.shape).tolist()
        self.initialize_spline()

    def timeReparameterize(self, t):
        if t > self._duration:
            return 1.0
        return 0.5 * (1 - np.cos(np.pi * t / self._duration))

    def getScaledDerivatives(self, t):
        t_ref = self.timeReparameterize(t)
        xds = self._traj.derivatives(t_ref, order=2)
        x = np.array(xds[0])
        v_raw = np.array(xds[1])
        a_raw = np.array(xds[2])
        v_scaling = 1 * np.pi / self._duration * np.sin(t * np.pi / self._duration)
        a_scaling = 1 * np.pi / self._duration * np.cos(t * np.pi / self._duration)
        v = v_scaling * v_raw / np.linalg.norm(v_raw)
        if np.linalg.norm(a_raw) < 1e-5:
            logging.warn(f"Assuming zero acceleration on spline, because of large magnitude {np.linalg.norm(a_raw)}")
            a = a_raw * 0
        else:
            a = a_scaling * a_raw / np.linalg.norm(a_raw)
        if t_ref == 1.0:
            v = v_raw * 0.0
            a = a_raw * 0.0
        return x, v, a

    def evaluate(self, t):
        x, v, a = self.getScaledDerivatives(t)
        return [x, v, a]

    def render(self):
        # Import Matplotlib visualization module
        from geomdl.visualization import VisMPL

        # Set the visualization component of the curve
        self._traj.vis = VisMPL.VisCurve3D()

        # Plot the curve
        self._traj.render()
        # TODO: The programm is basically stopped here <09-12-21, mspahn> #
