import cv2 as cv
import os
import numpy as np
from cv_bridge import CvBridge
from raubase_ros.interface import CVImage, ImageProcessingUnit, toBGR
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from ament_index_python import get_package_share_directory


class BallDetector(ImageProcessingUnit):
    """
    Detect golf ball with color bounded filtering.
    """

    # Computations
    KERNEL = np.ones((1, 1), np.uint8)
    BALL_RADIUS = 5
    QOS = 10

    # Debug configuration
    DEBUG_HSV = "ball_mask"
    CIRCLE_COLOR = (0, 255, 0)
    CIRCLE_WIDTH = 2

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()

    def setup(self, node: Node) -> None:
        # Initialize parameters
        self._lower_ball = toBGR(
            node.declare_parameter("ball_lower", "64640A")  # In RGB
            .get_parameter_value()
            .string_value
        )
        self._upper_ball = toBGR(
            node.declare_parameter("ball_upper", "FFFF14")  # In RGB
            .get_parameter_value()
            .string_value
        )

        # Initialize publishers
        self._msk_img_pub = node.create_publisher(
            Image, BallDetector.DEBUG_HSV, BallDetector.QOS
        )

    def run(
        self,
        img: CVImage,
        print_debug: bool = False,
        debug_img: CVImage | None = None,
    ) -> None:
        # Computing color bounded image mask
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.dilate(
            cv.inRange(hsv, self._lower_ball, self._upper_ball),
            BallDetector.KERNEL,
            iterations=2,
        )

        # Looking for contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv.contourArea)
            (x, y), radius = cv.minEnclosingCircle(c)
            if radius > BallDetector.BALL_RADIUS:  # RADIUS OF THE BALL

                # Debug display
                if print_debug:
                    cv.circle(
                        debug_img,
                        (int(x), int(y)),
                        int(radius),
                        BallDetector.CIRCLE_COLOR,
                        BallDetector.CIRCLE_WIDTH,
                    )
                # return (int(x), int(y), int(radius))

        if print_debug:
            self._msk_img_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="8UC1"))


class BallDetectorLearned(ImageProcessingUnit):
    """
    Detect golf ball with trained AI
    """

    # Computations
    QOS = 10

    # Debug configuration
    RECT_COLOR = (0, 255, 0)
    RECT_WIDTH = 2

    TEXT_COLOR = (255, 255, 255)
    TEXT_THICK = 2

    def __init__(self, model: str = "ball_detection"):
        super().__init__()

        # Get
        self.model: YOLO = self.load_model(model)

    def load_model(self, model_name: str) -> YOLO | None:
        # Get model file
        share_dir = get_package_share_directory("dtu_robocup_24")
        file_path = os.path.join(share_dir, "model", f"{model_name}.pt")

        if os.path.exists(file_path):
            return YOLO(file_path)
        else:
            return None

    def setup(self, node: Node) -> None:
        return

    def run(
        self,
        img: CVImage,
        print_debug: bool = False,
        debug_img: CVImage | None = None,
    ) -> None:
        # If the model is not loaded, stop here
        if self.model is None:
            return

        # Else compute results from model
        rgb_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        results = self.model([rgb_img], stream=True)

        if results is None:
            return

        # Exploit results
        for result in results:
            # Access the bounding box coordinates directly using the xyxy property
            # Assuming result.boxes is accessible and xyxy gives us the bounding box coordinates
            # Accessing confidence scores for each bounding box
            # Accessing class IDs for each bounding box
            [boxes, confs, classes] = [
                result.boxes.xyxy,
                result.boxes.conf,
                result.boxes.cls,
            ]

            # Iterate through each bounding box
            for box, conf, cls_id in zip(boxes, confs, classes):
                if conf > 0.6:  # Filter boxes with confidence > 0.6
                    x_min, y_min, x_max, y_max = map(int, box)

                    if print_debug:
                        # Draw the bounding box on the frame
                        cv.rectangle(
                            debug_img,
                            (x_min, y_min),
                            (x_max, y_max),
                            BallDetectorLearned.RECT_COLOR,
                            BallDetectorLearned.RECT_WIDTH,
                        )

                        # Get and draw the class name and confidence
                        class_name = result.names[int(cls_id)]
                        cv.putText(
                            debug_img,
                            f"{class_name} {conf:.2f}",
                            (x_min, y_min - 10),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            BallDetectorLearned.TEXT_COLOR,
                            BallDetectorLearned.TEXT_THICK,
                        )
