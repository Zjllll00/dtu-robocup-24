#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from rclpy import init, ok, shutdown, spin
from rclpy.node import Node
from raubase_msgs.msg import YoloResults, CmdMove
from sensor_msgs.msg import CameraInfo
from time import time
import math


class PlanThreegate(Node):

    YOLO_SUB_TOPIC = "yolo"
    MOVE_TOPIC = "move"
    CAMERA_INFO_TOPIC = "camera_info"

    def __init__(self) -> None:
        super().__init__("plan_template")

        self.yolo_results = YoloResults()
        self.sub = self.create_subscription(
            YoloResults, PlanTemplate.YOLO_SUB_TOPIC, self.yolo_result_callback, 10
        )
        self.cam_info = CameraInfo()
        self.cam_info_sub = self.create_subscription(
            CameraInfo, PlanTemplate.CAMERA_INFO_TOPIC, 10
        )
        self.move_cmd = CmdMove()
        self.move_cmd.move_type = CmdMove.CMD_V_TR
        self.pub = self.create_publisher(CmdMove, PlanTemplate.MOVE_TOPIC, 10)

        self.state = 100
        self.time = time()

    def yolo_result_callback(self, res: YoloResults):
        self.yolo_results = res

    def cam_info_callback(self, msg: CameraInfo):
        self.cam_info = msg
        
    def rotate(self, turn_rate, duration):
        """
        rotate the robot for a customized time.
        """
        self.move_cmd.velocity = 0.0 
        self.move_cmd.turn_rate = turn_rate  
        sleep(duration)  # wait for a while
        self.move_cmd.turn_rate = 0.0  # Stop rotating.
        sleep(1)  # Wait a moment before the next rotation (optional, adjust as needed).
    
    def caldist(class_id,rect):
        if class_id == 'ball':
            real_world_diameter_cm = 4
        elif class_id == 'house':
            real_world_diameter_cm = 10
        elif class_id == 'trolley':
            real_world_diameter_cm = 6
        else
            return None
        # Use cam info
        self.cam_info.k  # K matrix (3x3) of the camera
        self.cam_info.d  # d vector of the camera
        xmin, xmax, ymin, ymax = rect
        width = xmax - xmin
        height = ymax - ymin 
        # Calculate diameter in pixels as the average of width and height
        diameter_in_image_px = (width + height) / 2
 
        # Correct for camera distortion
        center_px = np.array([[[x + width / 2, y + height / 2]]], dtype=np.float32)
        undistorted_points = cv2.undistortPoints(center_px, self.cam_info.k, self.cam_info.d)
        x_corrected, y_corrected = undistorted_points[0,0,:]

        # Estimate the Z distance
        focal_length_px = (self.cam_info.k[0, 0] + self.cam_info.k[1, 1]) / 2
        Zc = (focal_length_px * real_world_diameter_cm) / diameter_in_image_px

        # Estimate X and Y based on Z
        Xc = (x_corrected - self.cam_info.k[0,2]) * Zc / self.cam_info.k[0, 0]
        Yc = (y_corrected - self.cam_info.k[1,2]) * Zc /self.cam_info.k[1, 1]
        return (Xc, Yc, Zc)
    
    def move_to_distance(Xc, Yc, Zc,target_distance):  
        turn_rate = 0.1 if Xc > 0 else -0.1  # rotate based on left or right
        rotation_angle = math.atan(abs(Xc) / Zc)  # calculate the rotation angle
        rotation_time = abs(rotation_angle) / turn_rate  # calculate the rotation time
        # implementing rotation
        self.move_cmd.velocity = 0.0
        self.move_cmd.turn_rate = turn_rate
        sleep(rotation_time)
        self.move_cmd.turn_rate = turn_rate

        distance_error = Zc - target_distance  # move distance
        velocity = 0.1 if distance_error > 0 else -0.1  # forward or backward
        move_time = abs(distance_error) / velocity  # calculate the moving time
        self.move_cmd.velocity = velocity
        self.move_cmd.turn_rate = 0.0
        sleep(move_time)
        self.move_cmd.velocity = 0.0


    def loop(self) -> None:
        # TODO: STATE MACHINE
        if state == 130:
        Print("Go to three gates")
        # foward 100 cm (corrected)the distance of line to the first gate
        self.move_cmd.turn_rate = 0.0
        self.move_cmd.velocity = 0.1
        sleep(10)#Speed and time need to adjust based on PID control
        self.move_cmd.velocity = 0.0
        # right 240 degree and foward a bit(corrected radius=34cm)
        self.move_cmd.turn_rate = -0.1
        sleep(2*math.pi/0.15+2)
        self.move_cmd.velocity = 0.051
        # adjust the pose
        Print("Back to the line")
        self.move_cmd.velocity = 0.0
        self.move_cmd.turn_rate = 0.1
        sleep(2+math.pi/0.2)
        self.move_cmd.turn_rate = 0.0
        # foward 65 cm (corrected) distance of line to the last gate
        self.move_cmd.velocity = 0.1
        sleep(6.5)
        break

if __name__ == "__main__":
    init()

    node = PlanTemplate()

    while ok():
        spin(node)

    shutdown()

