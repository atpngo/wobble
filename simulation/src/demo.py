import sim
import argparse
import numpy as np
import control
import util
import alt_control


def main():
    parser = argparse.ArgumentParser(
        description="Run balance trials with LQR, MPC or PID"
    )
    parser.add_argument(
        "-c",
        "--controller",
        choices=["lqr", "mpc", "pid"],
        default="lqr",
        help="which controller to use",
    )
    parser.add_argument(
        "-t", "--trials", type=int, default=3, help="number of trials to run"
    )
    parser.add_argument(
        "--max-runtime", type=float, default=10.0, help="max seconds per trial"
    )
    parser.add_argument(
        "--no-render", action="store_true", help="turn off GUI rendering"
    )
    parser.add_argument("--log-dir", default="../logs", help="where to write csv logs")
    args = parser.parse_args()

    # common system matrices for LQR & MPC
    A = np.array([[1.13995933, 7.92979201e-03], [1.14849707e02, 8.52681033e-01]])
    B = np.array([[-0.01098], [-1.104045]])
    q1, q2, r = 670, 110, 1000
    Q = np.diag([q1, q2])
    R = np.eye(1) * r

    # build controller wrapper
    if args.controller == "lqr":
        core = alt_control.LQRController(A, B, Q, R)
    elif args.controller == "mpc":
        N = 3
        core = alt_control.MPCController(A, B, Q, R, N)
    else:  # pid
        # PID(Kp, Ki, Kd, dt)
        dt = sim.Configuration.dt
        core = control.PID(10, 100, 0, dt)

    controller = alt_control.ControllerWrapper(core)

    render = not args.no_render
    for i in range(1, args.trials + 1):
        log_name = util.get_formatted_time_string(args.log_dir)
        exit_code, error = sim.run_trial(
            controller=controller,
            log_name=log_name,
            max_runtime=args.max_runtime,
            render=render,
            log_data=True,
        )
        print(
            f"[{args.controller.upper()}] Trial {i}: result={'FAIL' if exit_code == 1 else 'PASS'}, rmse={error:.3f}"
        )


if __name__ == "__main__":
    main()
