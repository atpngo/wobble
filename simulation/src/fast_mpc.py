import numpy as np
import sim
import alt_control
from multiprocessing import Pool, cpu_count

trials = 3
max_runtime = 10
max_N = 100


def eval_N(N):
    # build a fresh MPC + wrapper for this horizon
    A = np.array([[1.13995933, 7.92979201e-03], [1.14849707e02, 8.52681033e-01]])
    B = np.array([[-0.01098], [-1.104045]])
    q1, q2, r = 670, 110, 1000
    Q = np.diag([q1, q2])
    R = np.eye(1) * r
    core = alt_control.MPCController(A, B, Q, R, N)
    controller = alt_control.ControllerWrapper(core)

    rmses = []
    for _ in range(trials):
        # we don’t actually care about logs here
        exit_code, err = sim.run_trial(
            controller=controller,
            log_name="",
            max_runtime=max_runtime,
            render=False,
            log_data=False,
        )
        if exit_code != 0:
            # if any trial fails, discard this N
            return None
        rmses.append(err)

    return (N, sum(rmses) / len(rmses))


if __name__ == "__main__":
    # build list of horizons and dispatch
    horizons = list(range(1, max_N + 1))
    with Pool(processes=cpu_count()) as pool:
        results = pool.map(eval_N, horizons)

    # keep only those with all passes, sort by avg RMSE
    succeeded = [r for r in results if r is not None]
    succeeded.sort(key=lambda x: x[1])

    # report
    print(" N | avg RMSE")
    print("---+---------")
    for N, avg in succeeded:
        print(f"{N:2d} | {avg:.3f}")
