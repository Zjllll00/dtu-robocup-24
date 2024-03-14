import cv2 as cv
import cv2.aruco as ArUco
from raubase_ros.interface import CVImage, ImageProcessingUnit
from raubase_ros.wrappers import NodeWrapper
from rclpy.logging import get_logger
from raubase_msgs.msg import ArUcoObject, ResultArUco


class ArUcoProcessor(ImageProcessingUnit):
    """
    Detect ArUco markers in OpenCV Images.
    """

    ARUCO_TOPIC = "aruco"

    def __init__(self, aruco_dict: int = ArUco.DICT_4X4_250):
        super().__init__()
        self.__detector = ArUco.ArucoDetector(
            ArUco.getPredefinedDictionary(aruco_dict),
            ArUco.DetectorParameters(),
        )
        self.logger = get_logger("ArUco")

    def setup(self, node: NodeWrapper) -> None:
        self.aruco_msg = ResultArUco()
        self.aruco_pub = node.create_publisher(
            ResultArUco, ArUcoProcessor.ARUCO_TOPIC, 10
        )

    def run(
        self,
        img: CVImage,
        print_debug: bool = False,
        debug_img: CVImage | None = None,
    ) -> None:
        self.aruco_msg.detected = []

        # Get detected markers
        cnrs, ids, rjcts = self.__detector.detectMarkers(
            cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        )

        if ids is not None:
            for corners, id in zip(cnrs, ids):
                r = ArUcoObject()
                r.id = int(id)
                r.corners_x.resize((4))
                r.corners_y.resize((4))
                for i in range(4):
                    r.corners_x[i] = float(corners[0, i, 0])
                    r.corners_y[i] = float(corners[0, i, 1])

                r.x = 0.0
                r.y = 0.0
                r.z = 0.0
                self.aruco_msg.detected.append(r)

        self.aruco_pub.publish(self.aruco_msg)

        # DEBUG: draw them
        if print_debug:
            if ids is not None:
                ArUco.drawDetectedMarkers(debug_img, cnrs, ids)
            ArUco.drawDetectedMarkers(debug_img, rjcts, None, (255, 255, 20))
