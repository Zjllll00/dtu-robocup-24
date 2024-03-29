from time import sleep
from typing import List, Tuple
from raubase_ros.plan import BaseTask, close_to
from raubase_ros.plan.data import Requirement
from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    OnValue,
    StartTaskCondition,
)
from raubase_msgs.msg import ObjectYolo, ObjectArUco
import numpy as np

MESSAGE_THROTTLE = 1.0
LOOKING_TIME = 5.0
SERVO_INDEX = 1

TROLLEY_COLOR = {6: "green", 5: "red", 20: "yellow"}  # For logging purpose


class TaskStep(Enum):
    LAUNCH_TASK = auto()
    FIND_TROLLEY = auto()
    MOVE_TO_TROLLEY = auto()
    FIND_TROLLEY_CODE = auto()
    GRAB_TROLLEY = auto()
    DETECT_HOUSE = auto()
    MOVE_TO_HOUSE = auto()
    FIND_GREEN_HOUSE = auto()
    MOVE_TO_GREEN_HOUSE = auto()
    DROP_TROLLEY = auto()
    BACK_TO_LINE = auto()
    DONE = auto()


class PackageDelivery(BaseTask):

    LOOKING_TR = 0.3  # Looking around turn rate
    TROLLEY_CLASS = "trolley"  # The class to get for trolleys
    HOUSE_CLASS = "house"  # The class to get for houses
    MIN_CONF = 0.7  # The minimum confidence to have for YOLO results
    APPROACH_DIST = 0.02  # Approach distance in meters
    APPROACH_DIST_HOUSE = 0.3  # Approach distance to the house (in meters)

    ARM_POS_DOWN = -1024
    ARM_POS_UP = 1024
    SERVO_VEL = 80

    def __init__(self) -> None:
        super().__init__()
        self.state = TaskStep.LAUNCH_TASK
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

        # Internal registers for goals
        self.trolley_code = 0
        self.done_trolleys: List[int] = []
        self.trolley_goal: Tuple[float, float, float] | None = None
        self.house_goal: Tuple[float, float, float] | None = None
        self.initial_position: Tuple[float, float, float] = (0, 0, 0)

    def stop_condition(self):
        return self.stop_cond

    def requirements(self) -> Requirement:
        return (
            Requirement.MOVE
            | Requirement.ODOMETRY
            | Requirement.YOLO
            | Requirement.ARUCO
        )

    def start_conditions(self) -> StartTaskCondition:
        return FollowPreviousTask()

    # =========================================================================
    def move_to_distance(self, Xc: float, Zc: float, target_distance: float) -> bool:
        """
        Move to object position at a certain distance of the robot.

        Params:
            - Xc, Zc: the position in the camera frame of the object
            - target_distance: the targetted distance in meters that we want to have
        """
        # implementing rotation
        rot_angle = np.arctan2(Zc, Xc)  # calculate the rotation angle
        velocity = 0.2 if (Zc - target_distance) > 0 else -0.2  # forward or backward

        if close_to(rot_angle, 0) and close_to(Zc - target_distance, 0, 0.01):
            return True

        self.control.set_vel_h(velocity, rot_angle)
        return False

    def matching_ArUco(self, yolo: ObjectYolo, code: ObjectArUco) -> bool:
        """
        Check whether the given YOLO object encapsulate the provided ArUco code.

        I.E. if the ArUco code center is in the YOLO object bounding box.

        Params:
            - yolo: the detected yolo object
            - code: the detected ArUco code
        """
        cx = float((np.max(code.corners_x) + np.min(code.corners_x)) / 2)
        cy = float((np.max(code.corners_y) + np.min(code.corners_y)) / 2)
        return (
            yolo.xmin <= cx and cx <= yolo.xmax and yolo.ymin <= cy and cy <= yolo.ymax
        )

    # =========================================================================

    def loop(self) -> None:
        match self.state:
            case TaskStep.LAUNCH_TASK:
                """
                Initialize the task
                """

                self.logger.info("Initializing task ...")
                self.initial_position = (
                    self.data.odometry.x,
                    self.data.odometry.y,
                    self.data.odometry.heading,
                )
                self.data.reset_time()
                self.state = TaskStep.FIND_TROLLEY

            case TaskStep.FIND_TROLLEY:
                """
                Make the robot turn on himself to look for a trolley.
                """

                if len(self.done_trolleys) >= len(TROLLEY_COLOR.keys()):
                    self.state = TaskStep.BACK_TO_LINE

                self.logger.info(
                    "Looking for a trolley ...", throttle_duration_sec=MESSAGE_THROTTLE
                )
                # Turn around to find trolley
                self.control.set_vel_w(0, PackageDelivery.LOOKING_TR)

                # Get result from YOLO to find a trolley
                for trolley in self.data.last_yolo.detected:
                    assert type(trolley) is ObjectYolo
                    class_id = trolley.classifier
                    conf = trolley.confidence

                    if (
                        class_id == PackageDelivery.TROLLEY_CLASS
                        and conf > PackageDelivery.MIN_CONF
                    ):  # Detection of trolley
                        self.trolley_goal = (
                            trolley.robot_x.x,
                            trolley.robot_x.y,
                            trolley.robot_x.z,
                        )
                        self.state = TaskStep.MOVE_TO_TROLLEY
                        return

                # No trolley found, stop
                if self.data.time_elapsed >= LOOKING_TIME:
                    self.logger.error("No trolley found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()

            case TaskStep.MOVE_TO_TROLLEY:
                """
                Move the robot so that it can grab the trolley
                """

                self.logger.info(
                    "Moving towards the found trolley",
                    trottle_duration_sec=MESSAGE_THROTTLE,
                )
                # Sanity check if the goal exist, else go back to initial position
                if self.trolley_goal is None:
                    self.logger.error(
                        "Trying to move to the found trolley but did not saved its position internally ..."
                    )
                    self.state = TaskStep.BACK_TO_LINE
                # Else move towards the ball
                elif self.move_to_distance(
                    self.trolley_goal[0],
                    self.trolley_goal[2],
                    PackageDelivery.APPROACH_DIST,
                ):
                    self.data.reset_time()
                    self.state = TaskStep.FIND_TROLLEY_CODE

            case TaskStep.FIND_TROLLEY_CODE:
                """
                Reading the trolley code to go to the right house
                """

                self.logger.info(
                    "Looking for the trolley ArUco code ...",
                    trottle_duration_sec=MESSAGE_THROTTLE,
                )

                # Don't move the robot, but check if the ArUco code is visible
                for code in self.data.last_aruco.detected:
                    assert type(code) is ObjectArUco

                    # break the loop when detect aruco
                    if code.id in TROLLEY_COLOR.keys():
                        if code.id not in self.done_trolleys:
                            self.logger.info(
                                f"Found a trolley ArUco code: {code.id} -> is {TROLLEY_COLOR[code.id]} trolley ..."
                            )
                            self.trolley_code = code.id
                            self.trolley_goal = (code.x.x, code.x.y, code.x.z)
                            self.state = TaskStep.GRAB_TROLLEY
                            return

                # If no aruco found, stop
                if self.data.time_elapsed >= LOOKING_TIME:
                    self.logger.info("No code found for the trolley, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()

            case TaskStep.GRAB_TROLLEY:
                self.logger.info(
                    f"Grabbing {TROLLEY_COLOR[self.trolley_code]} trolley",
                    trottle_duration_sec=MESSAGE_THROTTLE,
                )
                self.control.set_servo(
                    SERVO_INDEX, PackageDelivery.ARM_POS_DOWN, PackageDelivery.SERVO_VEL
                )
                sleep(1.0)
                self.state = TaskStep.DETECT_HOUSE
                self.data.reset_time()

            case TaskStep.DETECT_HOUSE:
                """
                Detect the house matching the ArUco code of the package
                """

                # Turn around to find house
                self.logger.info(
                    f"Finding the house for {TROLLEY_COLOR[self.trolley_code]} trolley ...",
                    trottle_duration_sec=MESSAGE_THROTTLE,
                )
                self.control.set_vel_w(0, PackageDelivery.LOOKING_TR)

                # get result from YOLO
                houses = [
                    obj
                    for obj in self.data.last_yolo.detected
                    if obj.classifier == PackageDelivery.HOUSE_CLASS
                    and obj.confidence >= PackageDelivery.MIN_CONF
                ]
                # If we see all three houses, check for its ArUco code
                if len(houses) >= 3:
                    for house in houses:
                        assert type(house) is ObjectYolo

                        # Check for the corresponding ArUco code
                        for code in self.data.last_aruco.detected:
                            assert type(code) is ObjectArUco

                            if (
                                self.matching_ArUco(house, code)
                                and code.id == self.trolley_code
                            ):
                                self.logger.info(
                                    f"Found a matching house with ArUco code: {code.id} for {TROLLEY_COLOR[self.trolley_code]} trolley ..."
                                )
                                self.house_goal = (
                                    house.robot_x.x,
                                    house.robot_x.y,
                                    house.robot_x.z,
                                )
                                self.state = TaskStep.MOVE_TO_HOUSE
                                self.data.reset_distance()
                                return

                # Else if timeout, stop task
                elif self.data.time_elapsed >= LOOKING_TIME:
                    self.logger.info("No house found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()

            case TaskStep.MOVE_TO_HOUSE:
                """
                Moving towards the right house to drop the trolley.
                """

                self.logger.info(
                    "Moving towards the house area",
                    trottle_duration_sec=MESSAGE_THROTTLE,
                )
                if self.house_goal is None:
                    self.logger.error(
                        "Trying to move to the matching house but did not saved its position internally ..."
                    )
                    self.state = TaskStep.BACK_TO_LINE
                elif self.move_to_distance(
                    self.house_goal[0],
                    self.house_goal[2],
                    PackageDelivery.APPROACH_DIST_HOUSE,
                ):
                    self.state = TaskStep.DROP_TROLLEY

            case TaskStep.DROP_TROLLEY:
                """
                Drop the trolley in front of the house
                """

                self.logger.info(
                    "Dropping the green trolley",
                    trottle_duration_sec=MESSAGE_THROTTLE,
                )
                self.control.set_servo(
                    SERVO_INDEX, PackageDelivery.ARM_POS_UP, PackageDelivery.SERVO_VEL
                )

                self.data.reset_distance()
                self.control.set_vel_h(0.2, 0.0)

                if self.data.distance > 0.2:
                    self.logger.info(
                        "Taking some distance with the delivered trolley",
                        trottle_duration_sec=MESSAGE_THROTTLE,
                    )
                    self.done_trolleys.append(self.trolley_code)
                    self.data.reset_distance()
                    self.control.set_vel_h(0.0, 0.0)
                    self.state = TaskStep.FIND_TROLLEY
                    self.data.reset_time()

            case TaskStep.BACK_TO_LINE:
                """
                Moving back to original position so that we can grab the line
                """

                if self.move_to_distance(
                    self.data.odometry.x - self.initial_position[0],
                    self.data.odometry.y - self.initial_position[1],
                    0,
                ):
                    self.state = TaskStep.DONE

            case TaskStep.DONE:
                """
                Finishing the task
                """

                self.control.set_vel_w(0, 0)
                self.stop = True
