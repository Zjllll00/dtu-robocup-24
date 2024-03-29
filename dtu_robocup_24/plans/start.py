from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FlowTaskCondition,
    StartTaskCondition,
    FollowPreviousTask,
    StopTaskCondition,
    OnValue,
)
from raubase_ros.plan import BaseTask, Requirement


class TaskStep(Enum):
    START = auto()
    DONE = auto()


class StartTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()

        self.state = TaskStep.START
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        return self.stop_cond

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY | Requirement.LINE

    def loop(self) -> None:
        match self.state:
            case TaskStep.START:
                self.logger.info("start ...")
                self.control.follow_line(True, 0.3, 0.4)

                if self.data.distance >= 3:
                    self.state = TaskStep.DONE

            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.done = True
