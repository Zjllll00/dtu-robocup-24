from enum import Enum, auto
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


class TaskStep(Enum):
    TURN_DIR_RAMP = auto()
    GO_FOR_RAMP = auto()
    TURN_TO_RAMP = auto()
    BOARD_FORWARD = auto()
    TURN_TO_STAIRS = auto()
    GO_THROUGH_STAIRS = auto()
    DONE = auto()


class RampTask(BaseTask):
    def __init__(self) -> None:
        super().__init__("plan_template")

        self.state = TaskStep.TURN_DIR_RAMP
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

    def start_conditions(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_conditions(self) -> StopTaskCondition | FlowTaskCondition:
        return self.stop_cond

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY | Requirement.LINE

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
