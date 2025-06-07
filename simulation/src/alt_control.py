from scipy.linalg import solve_discrete_are
import numpy as np
import cvxpy as cp
import control


class ControllerWrapper:
    def __init__(self, controller):
        self.controller = controller
        self.type = type(controller)

    def get_signal(self, **kwargs):
        if self.type == control.PID:
            current_pitch, goal_pitch = kwargs["pitch"], kwargs["goal_pitch"]
            return self.controller.get_signal(current_pitch, goal_pitch)
        elif self.type == LQRController:
            pitch, delta_pitch = kwargs["pitch"], kwargs["d_pitch"]
            return (self.controller.control(np.asarray([pitch, delta_pitch])))[0]
        elif self.type == MPCController:
            pitch, delta_pitch = kwargs["pitch"], kwargs["d_pitch"]
            return self.controller.control(np.asarray([pitch, delta_pitch]))


class LQRController:
    """
    Discrete-time LQR:
      minimize sum (x'Qx + u'Ru)
      s.t. x_{k+1} = A x_k + B u_k

    u_k = -K x_k
    """

    def __init__(self, A, B, Q=None, R=None):
        self.A = np.asarray(A)
        self.B = np.asarray(B)
        n = self.A.shape[0]
        m = self.B.shape[1]

        # defaults if you don’t pass Q/R
        self.Q = np.eye(n) if Q is None else np.asarray(Q)
        self.R = np.eye(m) if R is None else np.asarray(R)

        # solve discrete‐time ARE: Aᵀ P A − P − (Aᵀ P B)(R + Bᵀ P B)⁻¹(Bᵀ P A) + Q = 0
        P = solve_discrete_are(self.A, self.B, self.Q, self.R)
        # compute K = (R + Bᵀ P B)⁻¹ (Bᵀ P A)
        self.K = np.linalg.inv(self.R + self.B.T @ P @ self.B) @ (self.B.T @ P @ self.A)

    def control(self, x):
        """
        x: state vector (n,)
        returns: control vector u (m,)
        """
        x = np.asarray(x).reshape(-1)
        return -self.K @ x

    def update_model(self, A=None, B=None, Q=None, R=None):
        """
        If you want to re-tune with new A/B/Q/R:
        """
        if A is not None:
            self.A = np.asarray(A)
        if B is not None:
            self.B = np.asarray(B)
        if Q is not None:
            self.Q = np.asarray(Q)
        if R is not None:
            self.R = np.asarray(R)
        # recompute K
        self.__init__(self.A, self.B, self.Q, self.R)


class MPCController:
    """
    Finite‐horizon MPC for x_{k+1}=A x_k + B u_k
    """

    def __init__(self, A, B, Q, R, N, x_min=None, x_max=None, u_min=None, u_max=None):
        self.A, self.B = np.asarray(A), np.asarray(B)
        self.Q, self.R, self.N = Q, R, N
        self.n, self.m = self.A.shape[0], self.B.shape[1]
        self.x_min, self.x_max = x_min, x_max
        self.u_min, self.u_max = u_min, u_max

        # decision vars
        self.x = cp.Variable((self.n, N + 1))
        self.u = cp.Variable((self.m, N))

        # parameter for current state
        self.x0 = cp.Parameter(self.n)

        cost = 0
        cons = [self.x[:, 0] == self.x0]
        for k in range(N):
            cost += cp.quad_form(self.x[:, k], self.Q) + cp.quad_form(
                self.u[:, k], self.R
            )
            cons += [self.x[:, k + 1] == self.A @ self.x[:, k] + self.B @ self.u[:, k]]
            if self.x_min is not None:
                cons += [self.x_min <= self.x[:, k], self.x[:, k] <= self.x_max]
            if self.u_min is not None:
                cons += [self.u_min <= self.u[:, k], self.u[:, k] <= self.u_max]
        # terminal cost
        cost += cp.quad_form(self.x[:, N], self.Q)

        self.prob = cp.Problem(cp.Minimize(cost), cons)

    def control(self, x_current):
        self.x0.value = x_current.reshape(-1)
        self.prob.solve(solver=cp.OSQP, warm_start=True)
        u0 = self.u[:, 0].value
        return u0 if u0 is not None else np.zeros(self.m)
