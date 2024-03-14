import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from raubase_ros.interface import CVImage, ImageProcessingUnit, toBGR
from rclpy.node import Node
from sensor_msgs.msg import Image
from raubase_msgs.msg import BallObject, ResultBalls


class BallProcessor(ImageProcessingUnit):
    """
    Detect golf ball with color bounded filtering.
    """

    BALL_TOPIC = "balls"

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
            Image,
            BallProcessor.DEBUG_HSV,
            BallProcessor.QOS,
        )
        self.ball_msg = ResultBalls()
        self._ball_pub = node.create_publisher(
            ResultBalls,
            BallProcessor.BALL_TOPIC,
            BallProcessor.QOS,
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
            BallProcessor.KERNEL,
            iterations=2,
        )

        self.ball_msg.detected = []

        # Looking for contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv.contourArea)
            (x, y), radius = cv.minEnclosingCircle(c)
            if radius > BallProcessor.BALL_RADIUS:  # RADIUS OF THE BALL
                r = BallObject()
                r.x, r.y, r.r = int(x), int(y), float(radius)
                self.ball_msg.detected.append(r)

                # Debug display
                if print_debug:
                    cv.circle(
                        debug_img,
                        (int(x), int(y)),
                        int(radius),
                        BallProcessor.CIRCLE_COLOR,
                        BallProcessor.CIRCLE_WIDTH,
                    )
                # return (int(x), int(y), int(radius))

        self._ball_pub.publish(self.ball_msg)

        if print_debug:
            self._msk_img_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="8UC1"))
