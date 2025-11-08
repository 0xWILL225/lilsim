"""
Path representation

Taken from the TSFS12 course at Linköping University, Sweden.
Original made by Erik Frisk. https://gitlab.liu.se/vehsys/tsfs12. 
Slightly modified.

Temporarily added in this repo for testing purposes. Will be removed later.
"""
import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import brenth
import warnings
from typing import Optional



class PathBase:
    def __init__(self, path: Optional[np.ndarray] = None) -> None:
        self._path = path
        self._computelength()

    @property
    def path(self) -> np.ndarray:
        """Points defining the path."""
        return self._path

    @path.setter
    def path(self, path: np.ndarray) -> None:
        self._path = path
        self._computelength()

    def _computelength(self) -> None:
        dp = self._path[0:-2, :] - self._path[1:-1, :]
        self.length = np.sum(np.sqrt(dp[:, 0]**2 + dp[:, 1]**2))


class SplinePath(PathBase):
    def __init__(self, points: np.ndarray, min_grid: Optional[float] = None) -> None:
        """Create a path object with spline interpolation.

            obj = SplinePath(points)

            Input:
                points - A Nx2 matrix with (x,y) coordinates on the path.

            Output:
                obj - SplinePath object


            Properties:
                path - property with the points defining the path

            Key methods:
                p - Get coordinates for a specific position on the path
                x - Get x-coordinate for a specific position on the path
                y - Get y-coordinate for a specific position on the path
                c - Get curvature at a specific position on the path
                heading - Get path tangential and normal vectors at a specific position on the path
                project - Project a general point onto the path
                path_error - Get orthogonal distance to the path at each time point for a given trajectory
              """
        si = np.hstack(([0], np.cumsum(np.sqrt(np.sum(np.diff(points[:, 0:2], axis=0)**2,
                                                      axis=1)))))
        if min_grid is not None:
            si_idx = [0]
            for k, si_k in enumerate(si):
                if si_k - si[si_idx[-1]] >= min_grid:
                    si_idx.append(k)
            if si[si_idx[-1]] != si[-1]:  # Always include last grid point
                si_idx.append(len(si) - 1)
            super(SplinePath, self).__init__(points[si_idx, 0:2])
        else:
            si_idx = np.arange(0, len(si))
            super(SplinePath, self).__init__(points[:, 0:2])

        si = si[si_idx]
        self.length = np.max(si)
        spline_type = 'quadratic'
        self.fx = interp1d(si, points[si_idx, 0], kind=spline_type)
        self.fy = interp1d(si, points[si_idx, 1], kind=spline_type)

        dfx = self.fx._spline.derivative()
        ddfx = dfx.derivative()
        dfy = self.fy._spline.derivative()
        ddfy = dfy.derivative()

        ci = (dfx(si) * ddfy(si) - dfy(si) * ddfx(si))

        self._dfx = dfx
        self._dfy = dfy
        self._c = interp1d(si, ci.reshape(-1), kind=spline_type, bounds_error=False, fill_value=(ci[0], ci[-1]))
        self._derc = self._c._spline.derivative()

    def p(self, s: float | np.ndarray) -> np.ndarray:
        """Compute point p(s)."""
        if np.isscalar(s):
            return np.array((self.fx(s), self.fy(s)))
        else:
            return np.column_stack((self.fx(s), self.fy(s)))

    def x(self, s: float | np.ndarray) -> float | np.ndarray:
        """Compute point x(s)."""
        return self.fx(s)

    def y(self, s: float | np.ndarray) -> float | np.ndarray:
        """Compute point y(s)."""
        return self.fy(s)

    def c(self, s: float | np.ndarray) -> float | np.ndarray:
        """Compute curvature c(s)."""
        return self._c(s)

    def der_c(self, s: float | np.ndarray) -> float | np.ndarray:
        """Compute derivative c'(s) of curvature c(s)."""

        return self._derc(s)

    def heading(self, s: float | np.ndarray[float]) -> tuple[np.ndarray, np.ndarray] | np.ndarray:
        """Return tangent and normal vector for path at point s

        h, nc = obj.heading(s_0)

        Returned vectors are normalized to length 1

        Arguments
        ---------
        s: float or np.ndarray[float]
            Position to evaluate heading

        Returns
        -------
        tuple(np.ndarray, np.ndarray) or np.ndarray
            (h, n_c) where
            h: Tangent vector
            n_c: Normal vector
        """
        h = np.hstack([self._dfx(s), self._dfy(s)])
        if np.isscalar(s):
            h = h / np.sqrt(h.dot(h))
            nc = np.array([-h[1], h[0]])

            return h, nc
        else:
            c = (1 / np.linalg.norm(h, axis=1)).reshape((-1, 1))
            h = c * h
            nc = np.column_stack((-h[:, 1], h[:, 0]))
            return h, nc

    def project(self, p: np.ndarray, s0: float, ds: float = 1, s_lim: int = 20, verbose: bool = False) -> tuple[float, float]:
        """Project a point on the path

           This is a line-search method to find an orthogonal
           projection of a point p on the path. This is a non-linear
           problem with in general does not have a unique solution;
           therefore an approximative approach is implemented.

             s, d = obj.project(p, s0, ds, s_lim)

            Arguments
            ---------
            p: np.ndarray
                The point to project
            s0: float
                Approximate position on the path (start of search)
            ds: float
                Step used to expand the search space (not equal to the accuracy of the projection)
            s_lim: int
                Number of expansions of the search space before admitting defeat.

            Returns
            -------
            tuple(float, float)
                (s, d) where
                s: Position on the path of the projection
                d: Distance between the point p and the projection
        """
        def s_fun(si):
            hi, nc = self.heading(si)
            dp = p - self.p(si)

            return float(dp[0] * nc[1] - dp[1] * nc[0])

        smin = s0
        smax = s0

        cnt_lim = s_lim / ds
        cnt = 0
        while np.sign(s_fun(smin)) == np.sign(s_fun(smax)) and cnt < cnt_lim:
            smin = np.max((0, smin - ds))
            smax = np.min((smax + ds, self.length))
            cnt = cnt + 1

        if cnt < cnt_lim:  # Found sign change in interval, do a line-search
            si = brenth(s_fun, smin, smax)
        else:  # No sign change, evaluate boundary points and choose closest
            if verbose:
                warnings.warn('Warning: Outside bounds')
            dpmin = p - self.p(smin)
            dpmax = p - self.p(smax)
            if dpmin.dot(dpmin) < dpmax.dot(dpmax):
                si = smin
            else:
                si = smax

        dp = p - self.p(si)
        hi, _ = self.heading(si)
        dp = np.cross(hi, dp)
        return si, dp

    def path_error(self, w: np.ndarray) -> np.ndarray:
        """Compute path error for trajectory

            d = obj.path_error(w)

            Input
              w - Trajectory where columns 1 and 2 are the x and y
                  coordinate of the vehicle path.

            Output
               d - Orthogonal distance to path at each time point."""

        N = w.shape[0]
        d = np.zeros(N)
        s0 = 0
        for k in range(0, N):
            p_car = w[k]
            si, dk = self.project(p_car, s0, ds=1, s_lim=20)
            d[k] = dk
            s0 = si
        return d

    def sample(self, N: Optional[int] = None, sample_length: Optional[float] = None) -> np.ndarray:
        """Sample the path at N points."""
        if N is not None:
            return self.p(np.linspace(0, self.length, N))
        elif sample_length is not None:
            return self.p(np.arange(0, self.length, sample_length))
        else:
            raise ValueError("Either N or sample_length must be provided")
