from raubase_ros.plan import BaseTask, close_to
from raubase_ros.plan.data import Requirement
from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    OnValue,
    StartTaskCondition,
)
import numpy as np



class TaskStep(Enum):
    LAUNCH_RACE_TRACK = auto()
    LEFT_90 = auto()
    FORWARD_50_1 = auto()
    RIGHT_90_1 = auto()
    FORWARD_60_1 = auto()
    RIGHT_90_2 = auto()
    FORWARD_50_2 = auto()
    RIGHT_90_3 = auto()
    FORWARD_60_2 = auto()
    ADJUST_POSE = auto()
    RACE_TRACK = auto()
    DONE = auto()
    
class ThreegateTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()
        self.state = TaskStep.CROSS_THREE_GATES
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

    def stop_condition(self):
        return self.stop_cond

    def requirements(self) -> Requirement:
        return (
            Requirement.MOVE
            | Requirement.ODOMETRY
        )

    def start_conditions(self) -> StartTaskCondition:
        return FollowPreviousTask()


    def loop(self) -> None:
        match self.state:
            case TaskStep.LAUNCH_RACE_TRACK:
                self.logger.info("Go to the start of race track...")
                self.data.reset_time()
                self.state = TaskStep.LEFT_90
            
            case TaskStep.LEFT_90:
            self.logger.info("Turn left 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,1.0)
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.FORWARD_50_1
                    
            case TaskStep.FORWARD_50_1:
                self.logger.info("Move forward 50cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.5
                    self.data.reset_distance()
                    self.state = TaskStep.RIGHT_90_1

            case TaskStep.RIGHT_90_1:
            self.logger.info("Turn right 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,-0.1)
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.FORWARD_60_1
                    
            case TaskStep.FORWARD_60_1:
                self.logger.info("Move forward 60cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.6
                    self.data.reset_distance()
                    self.state = TaskStep.RIGHT_90_2
                    
            case TaskStep.RIGHT_90_2:
            self.logger.info("Turn right 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,-0.1)
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.FORWARD_50_2
                    
            case TaskStep.FORWARD_50_2:
                self.logger.info("Move forward 50cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.5
                    self.data.reset_distance()
                    self.state = TaskStepRIGHT_90_3
                    
            case TaskStep.RIGHT_90_3:
            self.logger.info("Turn right 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,-0.1)
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.FORWARD_60_2
            
            case TaskStep.FORWARD_60_2:
                self.logger.info("Move forward 60cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.6
                    self.data.reset_distance()
                    self.state = TaskStep.ADJUST_POSE
                    
            case TaskStep.ADJUST_POSE:
            self.logger.info("Adjust the heading")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,-1.0)
                if self.data.time_elapsed >= (math.pi/4):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.RACE_TRACK
                    
            case TaskStep.RACE_TRACK:
                self.logger.info("pass the end gate...")
                self.data.reset_distance()
                self.control.set_vel_h(1.5,0.0)

                if self.data.distance > 4.5
                    self.data.reset_distance()
                    self.state = TaskStep.DONE
                    
            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.stop = True

if __name__ == "__main__":
    init()

    node = PlanTemplate()

    while ok():
        spin(node)

    shutdown()
