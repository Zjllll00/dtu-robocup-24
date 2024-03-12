import cv2 as cv
import cv2.aruco as ArUco
from raubase_ros.interface import CVImage, ImageProcessingUnit
from rclpy.node import Node


class ArUcoProcessor(ImageProcessingUnit):
    """
    Detect ArUco markers in OpenCV Images.
    """

    def __init__(self, aruco_dict: int = ArUco.DICT_4X4_250):
        super().__init__()
        self.__detector = ArUco.ArucoDetector(
            ArUco.getPredefinedDictionary(aruco_dict),
            ArUco.DetectorParameters(),
        )

    def setup(self, node: Node) -> None:
        return

    def run(
        self,
        img: CVImage,
        print_debug: bool = False,
        debug_img: CVImage | None = None,
    ) -> None:
        # Get detected markers
        cnrs, ids, rjcts = self.__detector.detectMarkers(
            cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        )

        # DEBUG: draw them
        if print_debug:
            if ids is not None:
                ArUco.drawDetectedMarkers(debug_img, cnrs, ids)
            ArUco.drawDetectedMarkers(debug_img, rjcts, None, (255, 255, 20))
