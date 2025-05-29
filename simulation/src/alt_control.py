from scipy.linalg import solve_discrete_are
import numpy as np


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
