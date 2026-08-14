#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from typing import List
import time
from enum import Enum
import math
from rclpy.impl.rcutils_logger import RcutilsLogger
from PCA9685 import PCA9685


class JointNum(Enum):
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    WRIST = 3
    HAND = 4
    GRIPPER = 5


NUM_SERVOS = JointNum.GRIPPER.value + 1


class Joint:
    """
    @brief A joint represents the state of a servo/joint keeping track of it's state
    configuration information and updating it's position in the real world.
    """

    ## --------------------------------------------------------------------------
    ## Tunable Parameters
    ## --------------------------------------------------------------------------

    ## These values can be updated based on your specific use case

    # How fast servos should move to target position between updates
    SMOOTHING_FACTOR = 0.075

    # Modifier for how fast a movement should be per joint
    SERVO_SPEEDS = [
        0.5,  # Servo 0: Base
        0.4,  # Servo 1: Shoulder
        0.4,  # Servo 2: Elbow
        0.7,  # Servo 3: Wrist Pitch (Swapped: Was Roll, now Pitch)
        1.5,  # Servo 4: Wrist Roll  (Swapped: Was Pitch, now Roll)
        1.5,  # Servo 5: Gripper
    ]

    # PWM min and max values per servo
    SERVO_PWM_LIMITS = [
        [602, 1012],  # Servo 0: Base
        [602, 1012],  # Servo 1: Shoulder
        [602, 1012],  # Servo 2: Elbow
        [705, 910],  # Servo 3: Wrist Pitch
        [705, 910],  # Servo 4: Wrist Roll
        [620, 930],  # Servo 5: Gripper
    ]

    SERVO_ANGLE_RANGE_DEG = [
        270,  # Servo 0: Base
        270,  # Servo 1: Shoulder
        270,  # Servo 2: Elbow
        180,  # Servo 3: Wrist Pitch
        180,  # Servo 4: Wrist Roll
        180,  # Servo 5: Gripper
    ]

    def __init__(self, joint: JointNum, min_pos: float, max_pos: float):
        self.joint = joint

        # Positioning info
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.center = max(self.min_pos, min(self.max_pos, 50.0))
        self.target = self.center
        self.current = self.center + self.SMOOTHING_FACTOR
        self.speed = self.SERVO_SPEEDS[joint.value]
        self.max_rad = math.radians(self.SERVO_ANGLE_RANGE_DEG[joint.value] / 2)

        # PWM info
        self.pwm_pin = joint.value
        self.min_pulse = self.SERVO_PWM_LIMITS[joint.value][0]
        self.max_pulse = self.SERVO_PWM_LIMITS[joint.value][1]

    def update(self):
        """
        Updates the Joint one tick and returns the new result
        """
        error = self.target - self.current
        self.current += error * self.SMOOTHING_FACTOR
        return self.current


class ArmController:
    """
    Manages the Baby Robot Arm movement.
    This will handle the limit.

    Note if no input commands are given after a self.SERVO_RELEASE_TIMEOUT_SEC the servo
    will be disabled to stop them from burning out.
    """

    def __init__(self, logger: RcutilsLogger, servo_min_limits: List[int], servo_max_limits: List[int]):

        # set the logger before anything else to ensure we can probably log messages.
        self._logger = logger

        self._joints: List[Joint] = [
            Joint(JointNum.BASE, servo_min_limits[JointNum.BASE.value], servo_max_limits[JointNum.BASE.value]),
            Joint(
                JointNum.SHOULDER,
                servo_min_limits[JointNum.SHOULDER.value],
                servo_max_limits[JointNum.SHOULDER.value],
            ),
            Joint(
                JointNum.ELBOW,
                servo_min_limits[JointNum.ELBOW.value],
                servo_max_limits[JointNum.ELBOW.value],
            ),
            Joint(
                JointNum.WRIST,
                servo_min_limits[JointNum.WRIST.value],
                servo_max_limits[JointNum.WRIST.value],
            ),
            Joint(
                JointNum.HAND,
                servo_min_limits[JointNum.HAND.value],
                servo_max_limits[JointNum.HAND.value],
            ),
            Joint(JointNum.GRIPPER, servo_min_limits[JointNum.GRIPPER.value], servo_max_limits[JointNum.GRIPPER.value]),
        ]

        self.get_logger().info(f"Loaded MIN limits: {servo_min_limits}")
        self.get_logger().info(f"Loaded MAX limits: {servo_max_limits}")

        self.servos_are_released = False
        # PCA9685 starts with global FULL_OFF set. The first update preloads
        # every channel before enabling the outputs.
        self.pwm_outputs_enabled = False
        self.last_input_time = time.time()

        # If min and max limits for servo positions is enabled.
        self._limits_enabled = True

        try:
            self.pwm = PCA9685()
            self.pwm.set_pwm_freq(50)
            self.get_logger().info("PCA9685 Initialized at 50Hz.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            self.get_logger().error("Is the I2C bus enabled and the device connected? Try running as root.")

            # rethrow the exception so the caller can handle graceful shutdown
            raise e

    ## --------------------------------------------------------------------------
    ## Main Hardware Control and Smoothing Loop
    ## --------------------------------------------------------------------------
    def update(self):
        """
        This is the main hardware loop. It runs at 50Hz and smoothly
        moves the servos to the self.target_positions, regardless of
        what controller set them.
        """

        # No point moving the servos
        if self.servos_are_released:
            return

        for joint in self._joints:
            new_percent = joint.update()

            # Check if an update is needed
            if new_percent is not None:
                self._set_percent(joint, new_percent)

        # All channel values are now valid, so it is safe to enable output.
        if not self.pwm_outputs_enabled:
            self.pwm.enable_all_pwm()
            self.pwm_outputs_enabled = True

    def move_joint(self, joint_num: JointNum, delta: float) -> float:
        """
        @brief Moves a servo based on it's current location.

        @note if the servo reaches the maximum it will be clamped at that location

        @return Resulting joint angle
        """
        if joint_num.value >= NUM_SERVOS or joint_num.value < 0:
            self.get_logger().warn("Tried to move invalid joint")
            return

        joint = self._joints[joint_num.value]

        # Use set_joint to handle the clamping
        return self.set_joint(joint_num, joint.target + delta * joint.speed)

    def set_joint(self, joint_num: JointNum, abs_pos: float) -> float:
        """
        @brief Moves a joint to an absolute location

        @return Resulting joint angle
        """
        if joint_num.value >= NUM_SERVOS or joint_num.value < 0:
            self.get_logger().warn("Tried to move invalid joint")
            return

        joint = self._joints[joint_num.value]

        # set_joint is used for both move and move_joint
        self.last_input_time = time.time()
        self.servos_are_released = False

        # The gripper always keeps its safety limits. IK may manage the limits
        # of the other five joints itself.
        if self._limits_enabled or joint_num == JointNum.GRIPPER:
            abs_pos = min(max(abs_pos, joint.min_pos), joint.max_pos)

        joint.target = abs_pos
        return joint.target

    def set_joint_rad(self, joint_num: JointNum, rad: float) -> float:
        """
        @brief Sets a joint angle with radians

        @note The value must be [-pi/2, pi/2] full 180 degree motion for servo 3-5
                           and [-3pi/4, 3pi/4] full 180 degree motion for servo 0-2
        @note 0 Is always assumed to be the origin

        @return joint's returned target value (Note this is not in radians)
        """

        joint = self._joints[joint_num.value]

        # Clamp the angle between the joint expected range of motion and normalized to [0,max_rad]
        # where max_rad is either pi/2 or 3pi/4 depending on the joint
        normalized_rad = max(min(joint.max_rad, rad), -joint.max_rad) + joint.max_rad

        # Convert to an absolute percent depending on the joint
        percent = normalized_rad / (joint.max_rad * 2) * 100

        # Now set joint will clamp it to the expected value if out of range
        self.get_logger().debug(f"Setting joint({joint_num.name}) to {rad}rad -> {percent}%")
        return self.set_joint(joint_num, percent)

    def get_joint_rad(self, joint_num: JointNum) -> float:
        """
        @brief Gets a joint angle in radians.

        @param joint_num Target joint.

        @return joint's current position in radians.
        """
        joint = self._joints[joint_num.value]

        # First we calculate the joints position from the percent in the shifted range [0, 2 * max_rad].
        # This is done so we don't need to worry about if the joint is positive or negative during the calculation.
        normalized_rad = (joint.current ) * (joint.max_rad * 2) / 100.0

        # Now shift the position value from [0, 2 * max_rad] back to our expected range [-max_rad, max_rad]
        rad = normalized_rad - joint.max_rad
        self.get_logger().debug(f"getting joint({joint_num.name}) to {rad}rad")
        return rad

    def center_all_servos(self):
        """
        @brief centers all servos, respecting the safe limits.
        """
        for joint in self._joints:
            self.set_joint(joint.joint, joint.center)

    def release_all_servos(self):
        """
        @brief Fully disables PWM signals to all servos, allowing them to go limp.

        The target and current positions remain unchanged so normal control can
        resume from the last commanded state when new input is received.
        """
        self.get_logger().info("Releasing all servos (turning off PWM).")
        self.servos_are_released = True
        self.pwm.disable_all_pwm()
        self.pwm_outputs_enabled = False

    def is_servos_centered(self):
        """
        @brief Checks if the servos or centered based on epsilon of SMOOTHING_FACTOR

        @return True if centered, otherwise False.
        """
        for joint in self._joints:
            if math.fabs(joint.current - joint.center) > joint.SMOOTHING_FACTOR:
                return False
        return True

    def enable_position_limits(self, enabled: bool) -> None:
        """
        @brief Sets if the servo should use self.min_pos and self.max_pos

        @param enabled If the position limits should be respected.
        """
        self._limits_enabled = enabled

    def get_logger(self) -> RcutilsLogger:
        return self._logger

    ## --------------------------------------------------------------------------
    ## Hardware Helper Functions
    ## --------------------------------------------------------------------------

    def _set_percent(self, joint: Joint, percentage) -> None:
        """
        @brief Converts a percentage (0-100) to a PWM value and sends it to a servo.
        """
        percentage = max(0, min(100, percentage))
        pwm_val = int(joint.min_pulse + (percentage / 100.0) * (joint.max_pulse - joint.min_pulse))
        self.pwm.set_pwm(joint.pwm_pin, 500, pwm_val)
