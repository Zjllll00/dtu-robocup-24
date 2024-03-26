from enum import Enum, auto
from raubase_ros.plan import BaseTask
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    StartTaskCondition,
    StopTaskCondition,
    Never,
)
from raubase_ros.plan.data import Requirement
import numpy as np


class State(Enum):
    MOVE_NORTH = auto()
    TURN_WEST = auto()
    MOVE_WEST = auto()
    TURN_SOUTH = auto()
    MOVE_SOUTH = auto()
    TURN_EAST = auto()
    MOVE_EAST = auto()
    TURN_NORTH = auto()


class TestTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()
        self.state: State = State.TURN_NORTH

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY

    def start_condition(self) -> StartTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition:
        return Never()

    def loop(self) -> None:
        # self.logger.info("Loop iteration")
        match self.state:
            case State.MOVE_NORTH:
                self.control.set_vel_h(0.3, np.pi / 2)

                self.logger.info(f"Distance = {self.data.distance}")
                if self.data.distance >= 1.5:
                    self.logger.info("Turning West !")
                    self.data.reset_distance()
                    self.state = State.TURN_WEST
            case State.TURN_WEST:
                self.control.set_vel_h(0, -np.pi)

                if close_to(self.data.odometry.heading, -np.pi):
                    self.logger.info("Moving West !")
                    self.data.reset_distance()
                    self.state = State.MOVE_WEST
            case State.MOVE_WEST:
                self.control.set_vel_h(0.3, -np.pi)

                if self.data.distance >= 1.5:
                    self.logger.info("Turning South !")
                    self.data.reset_distance()
                    self.state = State.TURN_SOUTH
            case State.TURN_SOUTH:
                self.control.set_vel_h(0, -np.pi / 2)

                if close_to(self.data.odometry.heading, -np.pi / 2):
                    self.logger.info("Moving South !")
                    self.data.reset_distance()
                    self.state = State.MOVE_SOUTH
            case State.MOVE_SOUTH:
                self.control.set_vel_h(0.3, -np.pi / 2)

                if self.data.distance >= 1.5:
                    self.logger.info("Turning East !")
                    self.data.reset_distance()
                    self.state = State.TURN_EAST
            case State.TURN_EAST:
                self.control.set_vel_h(0, 0)

                if close_to(self.data.odometry.heading, 0):
                    self.logger.info("Moving East !")
                    self.data.reset_distance()
                    self.state = State.MOVE_EAST
            case State.MOVE_EAST:
                self.control.set_vel_h(0.3, 0)

                if self.data.distance >= 1.5:
                    self.logger.info("Turning North !")
                    self.data.reset_distance()
                    self.state = State.TURN_NORTH
            case State.TURN_NORTH:
                self.control.set_vel_h(0, np.pi / 2)

                if close_to(self.data.odometry.heading, np.pi / 2):
                    self.logger.info("Moving North !")
                    self.data.reset_distance()
                    self.state = State.MOVE_NORTH
