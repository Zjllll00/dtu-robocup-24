#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from typing import Tuple
from raubase_ros.plan import BaseTask, close_to
from raubase_ros.plan.data import Requirement
from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    OnValue,
    StartTaskCondition,
)
import numpy as np

MESSAGE_THROTTLE = 1.0
LOOKING_TIME = 5.0
index = 1

class TaskStep(Enum):
    LAUNCH_TROLLEY_FINDING = auto()
    #FIRST ROUND FOR GREEN
    FIND_TROLLEY_1st = auto()
    MOVE_TO_TROLLEY_1st = auto()
    FIND_GREEN_TROLLEY = auto()
    GRAB_GREEN_TROLLEY = auto()
    DETECT_HOUSE_1st = auto()
    MOVE_TO_HOUSE_1st = auto()
    FIND_GREEN_HOUSE = auto()
    MOVE_TO_GREEN_HOUSE = auto()
    DROP_GREEN_TROLLEY = auto()
    #SECOND ROUND FOR RED
    FIND_TROLLEY_2nd = auto()
    MOVE_TO_TROLLEY_2nd = auto()
    FIND_RED_TROLLEY = auto()
    GRAB_RED_TROLLEY = auto()
    DETECT_HOUSE_2nd = auto()
    MOVE_TO_HOUSE_2nd = auto()
    FIND_RED_HOUSE = auto()
    MOVE_TO_RED_HOUSE = auto()
    DROP_RED_TROLLEY = auto()
    #THIRD ROUND FOR YELLOW
    FIND_TROLLEY_3rd = auto()
    MOVE_TO_TROLLEY_3rd = auto()
    FIND_YELLOW_TROLLEY = auto()
    GRAB_YELLOW_TROLLEY = auto()
    DETECT_HOUSE_3rd = auto()
    MOVE_TO_HOUSE_3rd = auto()
    FIND_YELLOW_HOUSE = auto()
    MOVE_TO_YWLLOW_HOUSE = auto()
    DROP_YELLOW_TROLLEY = auto()
    BACK_TO_LINE = auto()
    DONE = auto()
    
class ThreegateTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()
        self.state = TaskStep.CROSS_THREE_GATES
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)
        self.trolley_goal:Tuple[float,float,float]|None=None
        self.house_goal:Tuple[float,float,float]|None=None
        self.initial_position:Tuple[float,float,float]|None=None
        self.end_position:Tuple[float,float,float]|None=None
        
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
    def move_to_distance(self, Xc, Zc, target_distance) -> bool:
        # implementing rotation
        rot_angle = np.atan(Xc / Zc)  # calculate the rotation angle
        velocity = 0.2 if (Zc - target_distance) > 0 else -0.2  # forward or backward

        if close_to(rot_angle, 0) and close_to(Zc - target_distance, 0, 0.01):
            return True

        self.control.set_vel_h(velocity, rot_angle)
        return False
    # =========================================================================
    
    def loop(self) -> None:
        match self.state
            case TaskStep.LUANCH_TROLLEY_FINDING:
                self.logger.info("looking for a trolley")
                self.initial_position = (self.data.odometry.x,self.data.odometry.y,self.data.odometry.heading)
                self.data.reset_time()
                self.state = TaskStep.FIND_TROLLEY_1st
                
            case TaskStep.FIND_TROLLEY_1st:
                #move around to find trolley
                self.control.set_vel_w(0,0.3)

                # get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    
                    if class_id == "trolley" and conf > 0.7:  #Detection of trolley
                        self.trolley_goal=(r.robot_x.x,r.robot_x.y,r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_TROLLEY_1st
                        break
                #No trolley found,stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No trolley found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
                    
            case TaskStep.MOVE_TO_TROLLEY_1st:
                self.logger.info("Moving towards the trolleys", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.trolley_goal[0],self.trolley_goal[2],0.02): #0.02 in meters
                    self.state = TaskStep.FIND_GREEN_TROLLEY
            
            case TaskStep.FIND_GREEN_TROLLEY:
                self.logger.info("Finding green trolley...")
                self.control.set_vel_w(0,0.3)
                for code in self.data.last_aruco:
                    id = code.id
                    #break the loop when detect aruco
                    if self.data.last_aruco.id == 6
                        self.trolley_goal=(code.x.x,code.x.y,code.x.z)
                        self.state = TaskStep.GRAB_GREEN_TROLLEY
                        break
                #If no aruco found, stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No green house trolley, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
                    
            case TaskStep.GRAB_GREEN_TROLLEY:
                self.logger.info("Grabbing green trolley", trottle_duration_sec=MESSAGE_THROTTLE,)
                self.cintrol.set_servo(index, -1024, 1024)
                self.state = TaskStep.DETECT_HOUSE_1st
                    
            case TaskStep.DETECT_HOUSE_1st:
                #move around to find house
                self.logger.info("Finding a house...")
                self.control.set_vel_w(0,0.3)

                # get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    
                    if class_id == "house" and conf > 0.7:  #Detection of trolley
                        self.house_goal=(r.robot_x.x,r.robot_x.y,r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_HOUSE_1st
                        break

                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No house found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
            
            case TaskStep.MOVE_TO_HOUSE_1st:
                self.logger.info("Moving towards the house area", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.house_goal[0],self.house_goal[2],0.3): #0.3 in meter
                    self.state = TaskStep.FIND_GREEN_HOUSE
                    
            case TaskStep.FIND_GREEN_HOUSE:
                self.logger.info("Finding green house...")
                self.control.set_vel_w(0,0.3)
                for code in self.data.last_aruco:
                    id = code.id
                    #break the loop when detect aruco
                    if self.data.last_aruco.id == 6
                        self.house_goal=(code.x.x,code.x.y,code.x.z)
                        self.state = TaskStep.MOVE_TO_GREEN_HOUSE
                        break
                #If no aruco found, stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No green house found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
            
            case TaskStep.MOVE_TO_GREEN_HOUSE:
                self.logger.info("Moving towards the green house", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.house_goal[0],self.house_goal[2],0.05): #0.0.5 in meter
                    self.state = TaskStep.DROP_GREEN_TROLLEY
                    
            case TaskStep.DROP_GREEN_TROLLEY:
                self.logger.info("Dropping the green trolley", trottle_duration_sec=MESSAGE_THROTTLE,)
                self.cintrol.set_servo(index, 1024, 1024)
                
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.2
                    self.data.reset_distance()
                    self.control.set_vel_h(0.0,0.0)
                    
                self.state = TaskStep.FIND_TROLLEY_2nd
            
            case TaskStep.FIND_TROLLEY_2nd:
                #move around to find trolley
                self.control.set_vel_w(0,0.3)

                # get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    
                    if class_id == "trolley" and conf > 0.7:  #Detection of trolley
                        self.trolley_goal=(r.robot_x.x,r.robot_x.y,r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_TROLLEY_2nd
                        break
                #No trolley found,stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No trolley found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
                    
            case TaskStep.MOVE_TO_TROLLEY_2nd:
                self.logger.info("Moving towards the trolleys", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.trolley_goal[0],self.trolley_goal[2],0.02): #0.02 in meters
                    self.state = TaskStep.FIND_RED_TROLLEY
            
            case TaskStep.FIND_RED_TROLLEY:
                self.logger.info("Finding RED trolley...")
                self.control.set_vel_w(0,0.3)
                for code in self.data.last_aruco:
                    id = code.id
                    #break the loop when detect aruco
                    if self.data.last_aruco.id == 5
                        self.trolley_goal=(code.x.x,code.x.y,code.x.z)
                        self.state = TaskStep.GRAB_RED_TROLLEY
                        break
                #If no aruco found, stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No green house trolley, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
                    
            case TaskStep.GRAB_RED_TROLLEY:
                self.logger.info("Grabbing RED trolley", trottle_duration_sec=MESSAGE_THROTTLE,)
                self.cintrol.set_servo(index, -1024, 1024)
                self.state = TaskStep.DETECT_HOUSE_2nd
                    
            case TaskStep.DETECT_HOUSE_2nd:
                #move around to find house
                self.logger.info("Finding a house...")
                self.control.set_vel_w(0,0.3)

                # get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    
                    if class_id == "house" and conf > 0.7:  #Detection of trolley
                        self.house_goal=(r.robot_x.x,r.robot_x.y,r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_HOUSE_2nd
                        break

                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No house found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
            
            case TaskStep.MOVE_TO_HOUSE_2nd:
                self.logger.info("Moving towards the house area", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.house_goal[0],self.house_goal[2],0.3): #0.3 in meter
                    self.state = TaskStep.FIND_RED_HOUSE
                    
            case TaskStep.FIND_RED_HOUSE:
                self.logger.info("Finding RED house...")
                self.control.set_vel_w(0,0.3)
                for code in self.data.last_aruco:
                    id = code.id
                    #break the loop when detect aruco
                    if self.data.last_aruco.id == 5
                        self.house_goal=(code.x.x,code.x.y,code.x.z)
                        self.state = TaskStep.MOVE_TO_RED_HOUSE
                        break
                #If no aruco found, stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No RED house found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
            
            case TaskStep.MOVE_TO_RED_HOUSE:
                self.logger.info("Moving towards the RED house", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.house_goal[0],self.house_goal[2],0.05): #0.05 in meter
                    self.state = TaskStep.DROP_RED_TROLLEY
                    
            case TaskStep.DROP_RED_TROLLEY:
                self.logger.info("Dropping the RED trolley", trottle_duration_sec=MESSAGE_THROTTLE,)
                self.cintrol.set_servo(index, 1024, 1024)
                
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.2
                    self.data.reset_distance()
                    self.control.set_vel_h(0.0,0.0)
                    
                self.state = TaskStep.FIND_TROLLEY_3rd
                
            case TaskStep.FIND_TROLLEY_3rd:
                #move around to find trolley
                self.control.set_vel_w(0,0.3)

                # get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    
                    if class_id == "trolley" and conf > 0.7:  #Detection of trolley
                        self.trolley_goal=(r.robot_x.x,r.robot_x.y,r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_TROLLEY_3rd
                        break
                #No trolley found,stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No trolley found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
                    
            case TaskStep.MOVE_TO_TROLLEY_3rd:
                self.logger.info("Moving towards the trolleys", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.trolley_goal[0],self.trolley_goal[2],0.02): #0.02 in meters
                    self.state = TaskStep.FIND_YELLOW_TROLLEY
            
            case TaskStep.FIND_YELLOW_TROLLEY:
                self.logger.info("Finding YELLOW trolley...")
                self.control.set_vel_w(0,0.3)
                for code in self.data.last_aruco:
                    id = code.id
                    #break the loop when detect aruco
                    if self.data.last_aruco.id == 20
                        self.trolley_goal=(code.x.x,code.x.y,code.x.z)
                        self.state = TaskStep.GRAB_YELLOW_TROLLEY
                        break
                #If no aruco found, stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No green house trolley, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
                    
            case TaskStep.GRAB_YELLOW_TROLLEY:
                self.logger.info("Grabbing green trolley", trottle_duration_sec=MESSAGE_THROTTLE,)
                self.cintrol.set_servo(index, -1024, 1024)
                self.state = TaskStep.DETECT_HOUSE_3rd
                    
            case TaskStep.DETECT_HOUSE_3rd:
                #move around to find house
                self.logger.info("Finding a house...")
                self.control.set_vel_w(0,0.3)

                # get result from YOLO
                for r in self.data.last_yolo:
                    class_id = r.classifier
                    conf = r.confidence
                    
                    if class_id == "house" and conf > 0.7:  #Detection of trolley
                        self.house_goal=(r.robot_x.x,r.robot_x.y,r.robot_x.z)
                        self.state = TaskStep.MOVE_TO_HOUSE_3rd
                        break

                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No house found, stopping task...")
                    self.state = TaskStep.DONE
                    self.data.reset_distance()
            
            case TaskStep.MOVE_TO_HOUSE_3rd:
                self.logger.info("Moving towards the house area", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.house_goal[0],self.house_goal[2],0.3): #0.3 in meter
                    self.state = TaskStep.FIND_YWLLOW_HOUSE
                    
            case TaskStep.FIND_YELLOW_HOUSE:
                self.logger.info("Finding YELLOW house...")
                self.control.set_vel_w(0,0.3)
                for code in self.data.last_aruco:
                    id = code.id
                    #break the loop when detect aruco
                    if self.data.last_aruco.id == 20
                        self.house_goal=(code.x.x,code.x.y,code.x.z)
                        self.state = TaskStep.MOVE_TO_YELLOW_HOUSE
                        break
                #If no aruco found, stop
                if self.data.time_elapsed >= LOOKINGTIME:
                    self.logger.info("No YELLOW house found, stopping task...")
                    self.state = TaskStep.BACK_TO_LINE
                    self.data.reset_distance()
            
            case TaskStep.MOVE_TO_YELLOW_HOUSE:
                self.logger.info("Moving towards the YELLOW house", trottle_duration_sec=MESSAGE_THROTTLE,)
                if self.move_to_distance(self.house_goal[0],self.house_goal[2],0.05): #0.05 in meter
                    self.state = TaskStep.DROP_YELLOW_TROLLEY
                    
            case TaskStep.DROP_YELLOW_TROLLEY:
                self.logger.info("Dropping the YELLOW trolley", trottle_duration_sec=MESSAGE_THROTTLE,)
                self.cintrol.set_servo(index, 1024, 1024)
                
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.2
                    self.data.reset_distance()
                    self.control.set_vel_h(0.0,0.0)
                    
                self.state = TaskStep.BACK_TO_LINE
                
            case TaskStep.BACK_TO_LINE:
                self.end_position = (self.data.odometry.x,self.data.odometry.y,self.data.odometry.heading)
                def move_to_distance(self.end_position[0]-self.initial_position[0], self.end_position[1]-self.initial_position[1], 0)
                
            case TaskStep.Done:
                case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.stop = True
