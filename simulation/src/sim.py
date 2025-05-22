import sys, os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))

import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import random
import control


URDF_FILEPATH = os.path.join(script_dir, "robot.urdf")
FPS = 60
dt = 1 / FPS
# max_velocity = 100  # rad/sec prev 5
# max_force = 0.1  # prev 0.1
max_velocity = 100
max_force = 1


def add_noise(value):
    return random.uniform(-0.5, 0.5) + value


def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value


# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=90, cameraPitch=-5, cameraTargetPosition=[0, 0, 0]
)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load robot - replace with your URDF
initialOrientation = p.getQuaternionFromEuler([1 / 100, 0.0, 0])
robotId = p.loadURDF(URDF_FILEPATH, [0, 0, 0.2], initialOrientation)

# Get wheel joint indices
leftWheelIndex = 0
rightWheelIndex = 1

# Control parameters
# targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -100, 100, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 20, 10)

controller = control.PID(20, 0, 0.05, dt)
# Main simulation loop
while True:
    # Get robot state
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    robotEuler = p.getEulerFromQuaternion(robotOrn)
    pitch = robotEuler[0] * 180 / math.pi
    pitch = add_noise(pitch)

    # Simple P controller for balance
    balanceControl = controller.get_signal(pitch, 0.0)
    analogSignal = clamp(balanceControl, -255, 255)
    percentage = analogSignal / 255
    velocity = percentage * max_velocity
    left_wheel_velocity = -(velocity + random.uniform(-2, 2))
    right_wheel_velocity = velocity + random.uniform(-1, 1)
    if abs(pitch) > 45.0:
        left_wheel_velocity = 0.0
        right_wheel_velocity = 0.0
    left_encoder = p.getJointState(robotId, leftWheelIndex)[0]
    right_encoder = p.getJointState(robotId, rightWheelIndex)[0]
    # print(f"pitch: {pitch:.2f}, L: {left_encoder:.2f}, R: {right_encoder:.2f}")
    # Apply wheel controls
    p.setJointMotorControl2(
        robotId,
        leftWheelIndex,
        p.VELOCITY_CONTROL,
        targetVelocity=left_wheel_velocity,
        force=max_force,
    )
    p.setJointMotorControl2(
        robotId,
        rightWheelIndex,
        p.VELOCITY_CONTROL,
        targetVelocity=right_wheel_velocity,
        force=max_force,
    )

    p.resetDebugVisualizerCamera(
        cameraDistance=0.5, cameraYaw=45, cameraPitch=-10, cameraTargetPosition=robotPos
    )

    time.sleep(dt)
