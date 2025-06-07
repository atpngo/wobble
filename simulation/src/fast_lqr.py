import sim
import util
import alt_control
import time
import numpy as np
import itertools
from multiprocessing import Pool, cpu_count


A = np.array([[1.13995933e00, 7.92979201e-03], [1.14849707e02, 8.52681033e-01]])
B = np.array([[-0.01098], [-1.104045]])

# EXPANDED GRID
N = 10
grid = {
    "q1": np.linspace(0, 1000, N),
    "q2": np.linspace(0, 1000, N),
    "r": np.linspace(0, 1000, N),
}
params = list(itertools.product(grid["q1"], grid["q2"], grid["r"]))
total = len(params)
interval = max(1, total // 10)


def eval_lqr(param):
    q1, q2, r = param
    Q = np.diag([q1, q2])
    R = np.eye(1) * r
    try:
        raw = alt_control.LQRController(A, B, Q, R)
    except Exception:
        return None
    ctrl = alt_control.ControllerWrapper(raw)
    code, rmse = sim.run_trial(
        controller=ctrl,
        log_name=None,
        max_runtime=10,
        render=False,
        log_data=False,
    )
    return (q1, q2, r, code, rmse)


if __name__ == "__main__":
    start = time.time()
    passes = fails = 0
    best_rmse = float("inf")
    good = []

    with Pool(cpu_count()) as pool:
        for i, result in enumerate(pool.imap_unordered(eval_lqr, params), start=1):
            if i % interval == 0:
                elapsed = time.time() - start
                frac = i / total
                est = elapsed / frac
                left = est - elapsed
                print(
                    f"{frac:.0%} ({i}/{total}) elapsed={elapsed:.1f}s left≈{left:.1f}s best={best_rmse:.2f}"
                )

            if not result:
                fails += 1
                continue
            q1, q2, r, code, rmse = result
            best_rmse = min(best_rmse, rmse)
            if code == 0:
                passes += 1
                good.append((q1, q2, r, rmse))
            else:
                fails += 1

    print(
        f"Done {passes+fails} trials: passes={passes}, fails={fails}, best_rmse={best_rmse:.3f}, total={time.time()-start:.1f}s"
    )
    with open(util.get_relative_path("params.txt"), "w+") as f:
        for q1, q2, r, rmse in sorted(good, key=lambda x: x[3]):
            f.write(f"Q=({q1:.1e},{q2:.1e}) R={r:.1e}  RMSE={rmse:.3f}\n")
