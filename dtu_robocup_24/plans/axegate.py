from enum import Enum, auto
from raubase_ros.plan import BaseTask
from raubase_ros.plan.conditions import (
    FlowTaskCondition,
    StartTaskCondition,
    StopTaskCondition,
    AsSoonAsPossible,
    OnValue,
)
from raubase_ros.plan.data import Requirement


class TaskStep(Enum):
    GO_FORWARD = auto()
    STOP = auto()
    DONE = auto()


class AxeGateTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()

        self.state = TaskStep.GO_FORWARD
        self.stop = False
        self.stop_con = OnValue(lambda: self.stop)

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return AsSoonAsPossible()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        return self.stop_con

    def requirements(self) -> Requirement:
        return Requirement.DISTANCE | Requirement.MOVE

    # =========================================================================

    def loop(self) -> None:
        match self.state:
            case TaskStep.GO_FORWARD:
                self.control.follow_line(False, 0, 0.3)

                if self.data.distance >= 0.4:
                    self.state = TaskStep.DONE

                if self.data.ir[0].range < 0.1:
                    self.state = TaskStep.STOP

            case TaskStep.STOP:
                self.control.follow_line(False, 0, 0.0)

                if self.data.ir[0].range > 0.2:
                    self.state = TaskStep.GO_FORWARD

            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.stop = True
