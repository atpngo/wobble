import sys, os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))

import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import random
import datetime
import control

import numpy as np
from scipy.linalg import solve_discrete_are


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


URDF_FILEPATH = os.path.join(script_dir, "robot.urdf")

# Simulation params
dt = 0.01
max_velocity = 50  # radians per second
max_force = 1


def add_noise(value):
    return random.uniform(-0.5, 0.5) + value


def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value


class Logger:
    def __init__(self, filename, header):
        self.fout = open(filename, "w+")
        self.write(",".join(header))

    def __del__(self):
        print("Closing logger...")
        self.fout.close()

    def write(self, line):
        self.fout.write(line + "\n")


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

    def update(self, signal):
        """
        Step through and return state
        """
        # Emulate motor output velocity based on control signal
        analogSignal = clamp(signal, -255, 255)
        percentage = analogSignal / 255
        velocity = percentage * max_velocity
        left_wheel_velocity = -(velocity + random.uniform(-2, 2))
        right_wheel_velocity = velocity + random.uniform(-1, 1)
        left_wheel_velocity = -velocity
        right_wheel_velocity = velocity

        # Terminating condition is when the robot tips over
        if abs(self.get("pitch")) > 45.0:
            left_wheel_velocity = 0.0
            right_wheel_velocity = 0.0
            sys.exit(1)

        # Apply wheel force
        p.setJointMotorControl2(
            self.robot_id,
            self.left_wheel_idx,
            p.VELOCITY_CONTROL,
            targetVelocity=left_wheel_velocity,
            force=max_force,
        )
        p.setJointMotorControl2(
            self.robot_id,
            self.right_wheel_idx,
            p.VELOCITY_CONTROL,
            targetVelocity=right_wheel_velocity,
            force=max_force,
        )

        # Update state
        self.pos, self.ori = p.getBasePositionAndOrientation(self.robot_id)
        robotEuler = p.getEulerFromQuaternion(self.ori)
        pitch = robotEuler[0] * 180 / math.pi
        self.state["pitch"] = add_noise(pitch)
        self.state["left_enc"] = p.getJointState(self.robot_id, self.left_wheel_idx)[0]
        self.state["right_enc"] = p.getJointState(self.robot_id, self.right_wheel_idx)[
            0
        ]
        return self.state


class Simulation:
    def __init__(self):
        # Initialize PyBullet
        physicsClient = p.connect(p.GUI)
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
        p.setTimeStep(dt)

        # Load ground plane
        planeId = p.loadURDF("plane.urdf")

        # Load robot - replace with your URDF
        initialOrientation = p.getQuaternionFromEuler([1 / 100, 0.0, 0])
        self.robotId = p.loadURDF(URDF_FILEPATH, [0, 0, 0.2], initialOrientation)
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

    def step(self):
        p.stepSimulation()
        # Get robot state
        pitch = self.robot.get("pitch")

        # Control
        balanceControl = controller.get_signal(pitch, 0.0)

        self.robot.update(balanceControl)

        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=45,
            cameraPitch=-10,
            cameraTargetPosition=self.robot.get_position(),
        )
        robot_x, robot_y, robot_z = self.robot.get_position()
        p.addUserDebugText(
            f"Time: {self.current_time:.2f}s, Pitch: {pitch: .1f} deg",
            [robot_x - 0.8, robot_y, robot_z + 0.1],
            textColorRGB=[0, 0, 0],
            textSize=1.5,
            lifeTime=0,
            replaceItemUniqueId=self.time_text_id,
        )
        self.current_time += dt

        return self.current_time, self.robot.get_state()


# Use the simulation
class Environment:
    def __init__(self, timeout=10):
        self.timeout = timeout

    def step(self, left_wheel_signal, right_wheel_signal):
        pass


class ControllerWrapper:
    def __init__(self, controller):
        self.controller = controller
        self.type = type(controller)
        print(f"Using controller: {self.type}")

    def get_signal(self, arg1, arg2):
        if self.type == control.PID:
            current_pitch, goal_pitch = arg1, arg2
            return self.controller.get_signal(current_pitch, goal_pitch)
        elif self.type == LQRController:
            return (self.controller.control(np.asarray([arg1, arg2])))[0]
        elif self.type == control.LQR:
            pitch, delta_pitch = arg1, arg2
            return self.controller.compute(pitch, delta_pitch)


if __name__ == "__main__":
    now = datetime.datetime.now()
    ts = now.strftime("%m_%d_%Y_%H_%M_%S")
    logger = Logger(f"./logs/PID/trial_{ts}.log", ["timestamp_s", "pitch_degrees"])
    sim = Simulation()
    controller = ControllerWrapper(control.PID(10, 100, 0, dt))
    # controller = ControllerWrapper(control.LQR(1, 2.3))
    # dt = 1 / 60
    # A = np.array([[1, dt], [0, 1]])
    # B = np.array([[0], [dt]])
    # Q = np.diag([2, 1])
    # R = np.array([[0.5]])
    # controller = ControllerWrapper(LQRController(A, B, Q, R))
    while True:
        current_time_seconds, state = sim.step()
        logger.write(f"{current_time_seconds},{state['pitch']}")
        if current_time_seconds >= 10:
            break
