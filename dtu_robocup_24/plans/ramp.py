from enum import Enum, auto
from typing import Tuple
from raubase_ros.plan.conditions import (
    FlowTaskCondition,
    StartTaskCondition,
    FollowPreviousTask,
    StopTaskCondition,
    OnValue,
)
from raubase_ros.plan import BaseTask, close_to
import numpy as np
from raubase_ros.plan.data import Requirement

MESSAGE_THROTTLE = 1.0
LOOKING_TIME = 2.0

class TaskStep(Enum):
    TURN_DIR_RAMP = auto()
    GO_FOR_RAMP = auto()
    TURN_TO_RAMP = auto()
    BOARD_FORWARD = auto()
    TURN_TO_GOLF = auto()
    MOVE_TO_GOLF = auto()
    FIND_GOLF = auto()
    GRAB_GOLF = auto()
    TO_CENTER = auto()
    MOVE_TO_HOLE = auto()
    RELEASE_GOLF = auto()
    BACK_TO_CENTER = auto()
    TURN_TO_STAIRS = auto()
    GO_THROUGH_STAIRS = auto()
    DONE = auto()


class RampTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()

        self.state = TaskStep.TURN_DIR_RAMP
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)
        self.golf_goal:Tuple[float,float,float]|None=None
        self.init_position:Tuple[float,float,float]|None=None
        self.now_position:Tuple[float,float,float]|None=None

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        return self.stop_cond

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY | Requirement.LINE | Requirement.YOLO
    
# =========================================================================

    def move_to_distance(self, Xc, Zc, target_distance) -> bool:
        # implementing rotation
        rot_angle = np.atan(Xc / Zc)  # calculate the rotation angle
        velocity = 0.2 if (Zc - target_distance) > 0 else -0.2  # forward or backward

        if close_to(rot_angle, 0) and close_to(Zc - target_distance, 0, 0.1):
            return True

        self.control.set_vel_h(velocity, rot_angle)
        return False

# =========================================================================
    def loop(self) -> None:

        match self.state:
            case TaskStep.TURN_DIR_RAMP:
                self.logger.info("Going for the ramp ...")
                self.control.set_vel_h(0, 0)

                if close_to(self.data.odometry.heading, 0):
                    self.data.reset_distance()
                    self.state = TaskStep.GO_FOR_RAMP
            case TaskStep.GO_FOR_RAMP:
                self.control.set_vel_w(0.5, 0)

                if self.data.distance >= 2.5:
                    self.state = TaskStep.TURN_TO_RAMP
            case TaskStep.TURN_TO_RAMP:
                self.control.set_vel_h(0, -np.pi / 2)

                if close_to(self.data.odometry.heading, -np.pi / 2):
                    self.data.reset_distance()
                    self.state = TaskStep.BOARD_FORWARD

            case TaskStep.BOARD_FORWARD:
                self.logger.info("Climbing ...")
                self.control.follow_line(True, 0.3)

                if self.data.distance >= 2.7:
                    self.state = TaskStep.TURN_TO_GOLF
                    
            case TaskStep.TURN_TO_GOLF
                self.logger.info("Launching golf ball ...")
                self.state = TaskStep.FIND_GOLF
                self.data.reset_time()
                self.init_position = (self.data.odometry.x,self.data.odometry.y,self.data.odometry.heading)
                
            case TaskStep.FIND_GOLF
                self.logger.info("Finding golf ball ...")
                # Move around to find ball
                self.control.set_vel_w(0, 0.2)

                # Get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    if class_id == "white_ball" and conf > 0.7:  # Detection of ball
                        self.golf_goal = (r.robot_x.x, r.robot_x.y, r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_GOLF
                        break

                # If no ball found, stop
                if self.data.time_elapsed >= LOOKING_TIME:
                    self.logger.info("No ball found, stopping task ...")
                    self.state = TaskStep.BACK_TO_CENTER
                    self.data.reset_distance()
                    
            case TaskStep.MOVE_TO_GOLF
                self.logger.info(
                    "Moving towards the golf ball ...",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )
                # Move to the ball
                if self.move_to_distance(self.golf_goal[0], self.golf_goal[2], 2):
                    self.state = TaskStep.GRAB_BALL
                    
            case TaskStep.GRAB_GOLF
                self.logger.info(
                    "Grabbing the golf ball...",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )

                self.cintrol.set_servo(index, -1024, 1024)
                self.state = TaskStep.FIND_ARUCO

            case TaskStep.TO_CENTER
                self.logger.info("Finding center by odometry...")
                self.gnow_position = (self.data.odometry.x,self.data.odometry.y,self.data.odometry.heading)
                if self.move_to_distance(self.init_position[0]-self.now_position[0], self.init_position[1]-self.now_position[1], 0)
                    self.state = TaskStep.MOVE_TO_HOLE
                    
            case TaskStep.MOVE_TO_HOLE
                self.logger.info("Finding hole ...")
                if self.move_to_distance(0.2, 0.3, 0.05)
                    self.state = TaskStep.RELEASE_GOLF
                    
            case TaskStep.RELEASE_GOLF
                self.logger.info(
                    "release golf ball ...",
                    throttle_duration_sec=MESSAGE_THROTTLE,
                )

                self.cintrol.set_servo(index, 1024, 1024)
                self.state = TaskStep.BACK_TO_CENTER
                
            case TaskStep.BACK_TO_CENTER
                self.logger.info("back to center by odometry...")
                self.now_position = (self.data.odometry.x,self.data.odometry.y,self.data.odometry.heading)
                if self.move_to_distance(self.init_position[0]-self.now_position[0], self.init_position[1]-self.now_position[1], 0)
                    self.state = TaskStep.TURN_TO_STAIRS
                    
            case TaskStep.TURN_TO_STAIRS:
                self.logger.info("Going through the stairs ...")
                self.control.set_vel_h(0, np.pi / 2)

                if close_to(self.data.odometry.heading, np.pi / 2):
                    self.data.reset_time()
                    self.state = TaskStep.GO_THROUGH_STAIRS

            case TaskStep.GO_THROUGH_STAIRS:
                self.control.set_vel_w(0.5, 0)

                if self.data.time_elapsed >= 2:
                    self.state = TaskStep.DONE

            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.done = True
