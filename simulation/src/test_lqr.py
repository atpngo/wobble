import sim
import util
import alt_control
import numpy as np

A = np.array([[1.13995933e00, 7.92979201e-03], [1.14849707e02, 8.52681033e-01]])
B = np.array([[-0.01098], [-1.104045]])
q1 = 670
q2 = 110
r = 1000

Q = np.diag([q1, q2])
R = np.eye(1) * r
controller = sim.ControllerWrapper(alt_control.LQRController(A, B, Q, R))
# Run trials
for trial in range(3):
    exit_code, error = sim.run_trial(
        controller=controller,
        log_name=util.get_formatted_time_string("../logs"),
        max_runtime=10,
        render=True,
        log_data=False,
    )
