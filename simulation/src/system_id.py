import numpy as np
from sim import Simulation, Configuration


def f_step(env, x: np.ndarray, u: float) -> np.ndarray:
    state0 = {
        "pitch": float(x[0]),
        "pitch_rate": float(x[1]),
        "left_enc": 0.0,
        "right_enc": 0.0,
    }
    _, next_state = env.inject_and_step(state0, u, u)
    return np.array([next_state["pitch"], next_state["pitch_rate"]])


def identify_system(n_samples=2000):
    env = Simulation(render=False)
    X, U, Y = [], [], []

    for _ in range(n_samples):
        x0 = np.random.uniform(
            [-5, -50], [5, 50]
        )  # initialize random starting angle and angular velocity
        u = np.random.uniform(-255, 255)
        x1 = f_step(env, x0, u)
        X.append(x0)
        U.append([u])
        Y.append(x1)

    env.teardown()

    X = np.vstack(X)  # (N×2)
    U = np.vstack(U)  # (N×1)
    Y = np.vstack(Y)  # (N×2)
    Z = np.hstack([X, U])  # (N×3)

    M, *_ = np.linalg.lstsq(Z, Y, rcond=None)
    A = M[:2, :].T  # (2×2)
    B = M[2:, :].T  # (2×1)
    return A, B


if __name__ == "__main__":
    A, B = identify_system(n_samples=30000)
    print("Identified A:\n", A)
    print("Identified B:\n", B)
