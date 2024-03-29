from typing import Tuple
from raubase_ros.plan import BaseTask, close_to
from raubase_ros.plan.data import Requirement
from raubase_msgs.msg import ObjectArUco, ObjectYolo
from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    OnValue,
    StartTaskCondition,
)
import numpy as np

MESSAGE_THROTTLE = 1.0
LOOKING_TIME = 5.0
SERVO_INDEX = 1

class TaskStep(Enum):
    LAUNCH_BALL_FINDING = auto()
    FIND_BALL = auto()
    MOVE_TO_BALL = auto()
    GRAB_BALL = auto()
    FIND_ARUCO = auto()
    MOVE_BALL_TO_ARUCO = auto()
    DROP_BALL = auto()
    COUNT_BALL = auto()
    BACK_TO_START = auto()
    DONE = auto()


class MinigolfTask(BaseTask):
    ARM_POS_DOWN = -1024
    ARM_POS_UP = 1024
    SERVO_VEL = 80
    def __init__(self) -> None:
        super().__init__()
        self.state = TaskStep.LAUNCH_BALL_FINDING
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)
        self.n_balls = 0
        self.ball_goal: Tuple[float, float, float] | None = None
        self.aruco_goal: Tuple[float, float, float] | None = None
        self.initial_position: Tuple[float,float,float] | None = None

    def stop_condition(self):
        return self.stop_cond

    def requirements(self) -> Requirement:
        return (
            Requirement.MOVE
            | Requirement.ARUCO
            | Requirement.ODOMETRY
            | Requirement.YOLO
        )

    def start_condition(self) -> StartTaskCondition:
        return FollowPreviousTask()

    # =========================================================================

    def move_to_distance(self, Xc, Zc, target_distance) -> bool:
        # implementing rotation
        rot_angle = np.arctan2(Zc, Xc)  # calculate the rotation angle
        velocity = 0.2 if (Zc - target_distance) > 0 else -0.2  # forward or backward

        if close_to(rot_angle, 0) and close_to(Zc - target_distance, 0, 0.1):
            return True

        self.control.set_vel_h(velocity, rot_angle)
        return False

    # =========================================================================

    def loop(self) -> None:
        match self.state:
            case TaskStep.LAUNCH_BALL_FINDING:
                self.logger.info("Looking for a ball")
                self.initial_position = (
                    self.data.odometry.x,
                    self.data.odometry.y,
                    self.data.odometry.heading,
                )
                self.data.reset_time()
                self.state = TaskStep.FIND_BALL

            case TaskStep.FIND_BALL:
                # Move around to find ball
                self.control.set_vel_w(0, 0.3)

                # Get result from YOLO
                for r in self.data.last_yolo.detected:
                    assert type(r) is ObjectYolo
                    class_id = r.classifier
                    conf = r.confidence
                    if class_id == "orange_ball" and conf > 0.7:  # Detection of ball
                        self.ball_goal = (r.robot_x.x, r.robot_x.y, r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_BALL
                        break

                # If no ball found, stop
                if self.data.time_elapsed >= LOOKING_TIME:
                    self.logger.info("No ball found, stopping task ...")
                    self.state = TaskStep.BACK_TO_START
                    self.data.reset_distance()

            case TaskStep.MOVE_TO_BALL:
                self.logger.info(
                    "Moving towards the ball ...",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )
                # Move to the ball
                if self.ball_goal is None:
                    self.logger.error(
                        "Trying to move to ArUco code but did not saved its position internally ...",
                        throttle_duration_sec=MESSAGE_THROTTLE,
                    )
                    self.state = TaskStep.BACK_TO_START
                elif self.move_to_distance(self.ball_goal[0], self.ball_goal[2], 0.02):
                    self.state = TaskStep.GRAB_BALL

            case TaskStep.GRAB_BALL:
                self.logger.info(
                    "Grabbing the ball",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )

                self.control.set_servo(
                    SERVO_INDEX, MinigolfTask.ARM_POS_DOWN, MinigolfTask.SERVO_VEL
                )
                self.state = TaskStep.FIND_ARUCO

            case TaskStep.FIND_ARUCO:
                self.logger.info(
                    "Looking for a ball",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )

                # detect aruco
                self.control.set_vel_w(0, 0.2)

                # Get the result from aruco detect
                for code in self.data.last_aruco.detected:
                    assert type(code) is ObjectArUco
                    id = code.id

                    # break the loop when detect aruco
                    if id == 11 or id == 10:
                        self.aruco_goal = (code.x.x, code.x.y, code.x.z)
                        self.state = TaskStep.MOVE_BALL_TO_ARUCO
                        break

                # If no ArUco found, stop
                if self.data.time_elapsed >= LOOKING_TIME:
                    self.logger.info("No ArUco code found, stopping task ...")
                    self.state = TaskStep.BACK_TO_START
                    self.data.reset_distance()

            case TaskStep.MOVE_BALL_TO_ARUCO:
                self.logger.info(
                    "Moving towards the ArUco code ...",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )
                # Moving ball to the aruco
                if self.aruco_goal is None:
                    self.logger.error(
                        "Trying to move to ArUco code but did not saved its position internally ...",
                        throttle_duration_sec=MESSAGE_THROTTLE,
                    )
                    self.state = TaskStep.DONE
                elif self.move_to_distance(self.aruco_goal[0], self.aruco_goal[2], 0.10):
                    self.state = TaskStep.DROP_BALL

            case TaskStep.DROP_BALL:
                self.logger.info(
                    "Dropping the ball",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )

                self.control.set_servo(
                    SERVO_INDEX, MinigolfTask.ARM_POS_UP, MinigolfTask.SERVO_VEL
                )
                self.state = TaskStep.COUNT_BALL

            case TaskStep.COUNT_BALL:
                ball_count = len(
                    [
                        obj
                        for obj in self.data.last_yolo.detected
                        if obj.classifier == "orange_ball"
                    ]
                )
                if ball_count >= 4:
                    self.logger.info("Collected all 4 balls, we can stop")
                    self.state = TaskStep.BACK_TO_START
                else:
                    self.logger.info("We are still missing balls, let's try again")
                    self.state = TaskStep.MOVE_TO_BALL  # restart
                    
            case TaskStep.BACK_TO_START:
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
                self.control.set_vel_w(0, 0)
                self.stop = True
