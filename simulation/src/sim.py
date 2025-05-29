import sys, os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))

import pybullet as p
import pybullet_data
import math
import numpy as np
import control
import alt_control
import util
import numpy as np
import random

random.seed(69)


# Simulation params
class Configuration:
    urdf_filepath = os.path.join(script_dir, "robot.urdf")
    dt = 0.01
    max_velocity = 50  # radians per second
    max_force = 1


class Robot:
    def __init__(
        self,
        robot_id,
        left_wheel,
        right_wheel,
        initial_pitch=0.0,
        initial_left_enc=0,
        initial_right_enc=0,
    ):
        self.robot_id = robot_id
        self.left_wheel_idx = left_wheel
        self.right_wheel_idx = right_wheel
        self.state = {
            "left_enc": initial_left_enc,
            "right_enc": initial_right_enc,
            "pitch": initial_pitch,
        }
        self.pos = None
        self.ori = None

    def get(self, key):
        if key in self.state:
            return self.state[key]
        raise KeyError

    def get_state(self):
        return self.state

    def get_position(self):
        return self.pos

    def get_orientation(self):
        return self.ori

    def get_velocity_from_signal(self, signal):
        analog_signal = util.clamp(signal, -255, 255)
        percentage = analog_signal / 255
        return percentage * Configuration.max_velocity

    def update(self, left_motor_signal, right_motor_signal):
        """
        Step through and return state
        """
        # Emulate motor output velocity based on control signal
        left_velocity = self.get_velocity_from_signal(left_motor_signal)
        right_velocity = self.get_velocity_from_signal(right_motor_signal)

        left_wheel_velocity = -util.add_noise(left_velocity, 2)
        right_wheel_velocity = util.add_noise(right_velocity, 1)
        # left_wheel_velocity = -left_velocity
        # right_wheel_velocity = right_velocity

        # Apply wheel force
        p.setJointMotorControl2(
            self.robot_id,
            self.left_wheel_idx,
            p.VELOCITY_CONTROL,
            targetVelocity=left_wheel_velocity,
            force=Configuration.max_force,
        )
        p.setJointMotorControl2(
            self.robot_id,
            self.right_wheel_idx,
            p.VELOCITY_CONTROL,
            targetVelocity=right_wheel_velocity,
            force=Configuration.max_force,
        )

        # Update state
        self.pos, self.ori = p.getBasePositionAndOrientation(self.robot_id)
        robotEuler = p.getEulerFromQuaternion(self.ori)
        pitch = robotEuler[0] * 180 / math.pi
        self.state["pitch"] = util.add_noise(pitch, 0.5)
        self.state["left_enc"] = p.getJointState(self.robot_id, self.left_wheel_idx)[0]
        self.state["right_enc"] = p.getJointState(self.robot_id, self.right_wheel_idx)[
            0
        ]
        return self.state


class Simulation:
    def __init__(self, render):
        # Initialize PyBullet
        physicsClient = p.connect(p.GUI if render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(1)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=90,
            cameraPitch=-5,
            cameraTargetPosition=[0, 0, 0],
        )
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        p.setRealTimeSimulation(0)
        p.setTimeStep(Configuration.dt)

        # Load ground plane
        planeId = p.loadURDF("plane.urdf")

        # Load robot - replace with your URDF
        initialOrientation = p.getQuaternionFromEuler([1 / 100, 0.0, 0])
        self.robotId = p.loadURDF(
            Configuration.urdf_filepath, [0, 0, 0.2], initialOrientation
        )
        self.robot = Robot(self.robotId, initial_pitch=0.2, left_wheel=0, right_wheel=1)

        # Timing
        self.current_time = 0.0
        self.time_text_id = p.addUserDebugText(
            "Time: 0.00",
            [0.0, 0.0, 0.0],
            textColorRGB=[1, 1, 1],
            textSize=1.5,
            lifeTime=0,
        )

    def teardown(self):
        p.disconnect()

    def get_state(self):
        return self.current_time, self.robot.get_state()

    def step(self, left_motor_signal, right_motor_signal):
        p.stepSimulation()

        self.robot.update(left_motor_signal, right_motor_signal)

        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=45,
            cameraPitch=-10,
            cameraTargetPosition=self.robot.get_position(),
        )
        robot_x, robot_y, robot_z = self.robot.get_position()
        p.addUserDebugText(
            f"Time: {self.current_time:.2f}s, Pitch: {self.robot.get("pitch"): .1f} deg",
            [robot_x - 0.8, robot_y, robot_z + 0.1],
            textColorRGB=[0, 0, 0],
            textSize=1.5,
            lifeTime=0,
            replaceItemUniqueId=self.time_text_id,
        )
        self.current_time += Configuration.dt

        return self.get_state()


class ControllerWrapper:
    def __init__(self, controller):
        self.controller = controller
        self.type = type(controller)
        print(f"Using controller: {self.type}")

    def get_signal(self, **kwargs):
        if self.type == control.PID:
            current_pitch, goal_pitch = kwargs["pitch"], kwargs["goal_pitch"]
            return self.controller.get_signal(current_pitch, goal_pitch)
        elif self.type == alt_control.LQRController:
            pitch, delta_pitch = kwargs["pitch"], kwargs["d_pitch"]
            return (self.controller.control(np.asarray([pitch, delta_pitch])))[0]
        elif self.type == control.LQR:
            pitch, delta_pitch = kwargs["pitch"], kwargs["d_pitch"]
            return self.controller.compute(pitch, delta_pitch)


def run_trial(controller, log_name, max_runtime=10, render=True):
    sim = Simulation(render)
    logger = util.Logger(log_name, ["timestamp_s", "pitch_degrees"])

    try:
        # Main logic
        current_time_seconds, state = sim.get_state()
        logger.write(f"{round(current_time_seconds, 3)},{state['pitch']}")
        curr_pitch, prev_pitch = state["pitch"], 0
        while True:
            pitch = state["pitch"]
            d_pitch = pitch - prev_pitch
            prev_pitch = pitch
            signal = controller.get_signal(pitch=pitch, d_pitch=d_pitch, goal_pitch=0)
            left_control_signal = signal
            right_control_signal = signal
            current_time_seconds, state = sim.step(
                left_control_signal, right_control_signal
            )
            logger.write(f"{round(current_time_seconds, 3)},{round(state['pitch'],5)}")
            if current_time_seconds >= max_runtime:
                print("PASS")
                return 0

            # Terminating conditions
            if pitch > 45:
                print("FAIL")
                return 1
    finally:
        sim.teardown()


if __name__ == "__main__":
    # PID Controller
    controller = ControllerWrapper(control.PID(10, 100, 0, Configuration.dt))

    # LQR Controller
    # A = np.array([[1, Configuration.dt], [0, 1]])
    # B = np.array([[0], [Configuration.dt]])
    # Q = np.diag([1, 1])  # pitch angular position, pitch angular velocity
    # R = np.array([1])  # output torque
    # controller = ControllerWrapper(alt_control.LQRController(A, B, Q, R))

    # A = np.array([[1, Configuration.dt], [0, 1]])
    # B = np.array([[0], [Configuration.dt]])
    # grid = {
    #     "q1": [1, 10, 50, 100],
    #     "q2": [0.1, 1, 10],
    #     "r": [0.01, 0.1, 1],
    # }
    # Run trials
    for trial in range(3):
        # Q = np.diag([1, 1])  # pitch angular position, pitch angular velocity
        # R = np.array([1])  # output torque
        # controller = ControllerWrapper(alt_control.LQRController(A, B, Q, R))

        exit_code = run_trial(
            controller=controller,
            log_name=util.get_formatted_time_string("./logs"),
            max_runtime=5,
            render=False,
        )
    sys.exit(exit_code)
