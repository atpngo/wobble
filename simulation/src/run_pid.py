import sim
import util
import control

controller = sim.ControllerWrapper(control.PID(10, 100, 0, sim.Configuration.dt))
# Run trials
for trial in range(3):
    exit_code, error = sim.run_trial(
        controller=controller,
        log_name=util.get_formatted_time_string("../logs"),
        max_runtime=10,
        render=True,
        log_data=False,
    )
