from rclpy import init, ok, shutdown, spin
from rclpy.node import Node
from raubase_msgs.msg import YoloResults, CmdMove
from sensor_msgs.msg import CameraInfo


class PlanTemplate(Node):

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

        self.state = 0

    def yolo_result_callback(self, res: YoloResults):
        self.yolo_results = res

    def cam_info_callback(self, msg: CameraInfo):
        self.cam_info = msg

    def loop(self) -> None:
        # TODO: STATE MACHINE

        # Get yolo result:
        for r in self.yolo_results.detected:
            rect = (r.xmin, r.xmax, r.ymin, r.ymax)
            class_id = r.classifier
            conf = r.confidence

        # Move Robot
        self.move_cmd.velocity = 0.1
        self.move_cmd.turn_rate = 0.0

        # or
        self.move_cmd.velocity = 0.1
        self.move_cmd.heading = 0.0
        
        # Use cam info
        self.cam_info.k  # K matrix (3x3) of the camera
        self.cam_info.d  # d vector of the camera


if __name__ == "__main__":
    init()

    node = PlanTemplate()

    while ok():
        spin(node)

    shutdown()
