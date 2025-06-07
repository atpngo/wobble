import sys, os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))

import pybullet as p
import pybullet_data
import math
import util
import random
import time

random.seed(69)


# Simulation params
class Configuration:
    urdf_filepath = util.get_relative_path("robot.urdf")
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
            "pitch_rate": 0,
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

    def set_signal(self, left_motor_signal, right_motor_signal):
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

    def update_state(self):
        # Update state
        old_pitch = self.state["pitch"]
        self.pos, self.ori = p.getBasePositionAndOrientation(self.robot_id)
        robotEuler = p.getEulerFromQuaternion(self.ori)
        pitch = robotEuler[0] * 180 / math.pi
        self.state["pitch"] = util.add_noise(pitch, 0.5)
        self.state["left_enc"] = p.getJointState(self.robot_id, self.left_wheel_idx)[0]
        self.state["right_enc"] = p.getJointState(self.robot_id, self.right_wheel_idx)[
            0
        ]
        self.state["pitch_rate"] = (self.state["pitch"] - old_pitch) / Configuration.dt
        return self.state


class Simulation:
    def __init__(self, render):
        # Initialize PyBullet
        if p.isConnected():
            p.disconnect()
        self.client = p.connect(p.GUI if render else p.DIRECT)
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
        if p.isConnected():
            p.disconnect(self.client)

    def get_state(self):
        return self.current_time, self.robot.get_state()

    def step(self, left_motor_signal, right_motor_signal):

        self.robot.set_signal(left_motor_signal, right_motor_signal)

        p.stepSimulation()

        self.robot.update_state()

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

    def inject_and_step(self, state: dict, left_signal: float, right_signal: float):
        """
        state must contain:
        - "pitch"      in degrees
        - "pitch_rate" in deg/s
        - "left_enc"   in rad
        - "right_enc"  in rad
        """
        # 1) Reset the base pose to the same XYZ but desired pitch about X
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        quat = p.getQuaternionFromEuler(
            [
                math.radians(state["pitch"]),  # pitch
                0.0,  # roll
                0.0,  # yaw
            ]
        )
        p.resetBasePositionAndOrientation(self.robotId, pos, quat)

        # 2) Reset base angular velocity to match pitch_rate (deg/s → rad/s)
        omega = math.radians(state["pitch_rate"])
        p.resetBaseVelocity(
            self.robotId,
            linearVelocity=[0, 0, 0],
            angularVelocity=[omega, 0, 0],
        )

        # 3) Reset each wheel encoder to its angle
        p.resetJointState(
            self.robotId,
            self.robot.left_wheel_idx,
            state["left_enc"],
            targetVelocity=0.0,
        )
        p.resetJointState(
            self.robotId,
            self.robot.right_wheel_idx,
            state["right_enc"],
            targetVelocity=0.0,
        )

        # 4) Issue your control signals
        self.robot.set_signal(left_signal, right_signal)

        # 5) Step the world forward by one dt
        p.stepSimulation()

        self.robot.update_state()

        self.current_time += Configuration.dt

        # 6) Return the new sim time & robot state
        return self.get_state()


def run_trial(controller, log_name, max_runtime=10, render=True, log_data=True):
    sim = Simulation(render)
    logger = util.Logger(
        log_name,
        ["timestamp_s", "pitch_degrees", "control_latency_microseconds"],
        log_data=log_data,
    )
    pitch_history = []
    try:
        # Main logic
        current_time_seconds, state = sim.get_state()
        logger.write(f"{round(current_time_seconds, 3)},{state['pitch']}")
        curr_pitch, prev_pitch = state["pitch"], 0
        while True:
            pitch = state["pitch"]
            pitch_history.append(pitch)  # record pitch to compute RMSE
            d_pitch = (pitch - prev_pitch) / Configuration.dt
            prev_pitch = pitch

            before = time.time()
            signal = controller.get_signal(pitch=pitch, d_pitch=d_pitch, goal_pitch=0)
            after = time.time()

            left_control_signal = signal
            right_control_signal = signal
            current_time_seconds, state = sim.step(
                left_control_signal, right_control_signal
            )
            logger.write(
                f"{round(current_time_seconds, 3)},{round(state['pitch'],5)},{1e6*(after-before):.2f}"
            )
            if current_time_seconds >= max_runtime:
                error = util.rmse(pitch_history)
                if error < 10:
                    return 0, error
                else:
                    return 1, error

            # Terminating conditions
            if abs(pitch) > 45:
                return 1, util.rmse(pitch_history)
    finally:
        sim.teardown()


if __name__ == "__main__":
    pass
