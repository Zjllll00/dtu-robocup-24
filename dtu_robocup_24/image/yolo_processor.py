import cv2 as cv
import os
from raubase_ros.interface import CVImage, ImageProcessingUnit
from rclpy.logging import get_logger
from ultralytics import YOLO
from ament_index_python import get_package_share_directory
from raubase_ros.wrappers import NodeWrapper
from raubase_msgs.msg import ResultYolo, YoloObject


class MLObjectProcessor(ImageProcessingUnit):
    """
    Detect golf ball with trained AI
    """

    # ROS things
    YOLO_OUTPUT_TOPIC = "yolo"
    QOS = 10

    # Debug configuration
    RECT_COLOR = (0, 255, 0)
    RECT_WIDTH = 2

    TEXT_COLOR = (255, 255, 255)
    TEXT_THICK = 2

    def __init__(self):
        super().__init__()

        self.logger = get_logger("YOLOProcessor")

    def load_model(self, model_name: str) -> YOLO | None:
        # Get model file
        share_dir = get_package_share_directory("dtu_robocup_24")
        file_path = os.path.join(share_dir, "model", f"{model_name}.pt")

        if os.path.exists(file_path):
            self.logger.info(f"Loading model {model_name}")
            return YOLO(file_path)
        else:
            self.logger.error(f"Model {model_name} not found !")
            return None

    def setup(self, node: NodeWrapper) -> None:
        # Load model
        model_name = node.declare_wparameter("yolo_model", "ball_detection").get()
        self.model: YOLO | None = self.load_model(model_name)

        # If model loaded, set an output topic
        if self.model is not None:
            self.yolo_msg = ResultYolo()
            self.yolo_msg.detected = []
            self.yolo_pub = node.create_publisher(
                ResultYolo,
                MLObjectProcessor.YOLO_OUTPUT_TOPIC,
                10,
            )

    def run(
        self,
        img: CVImage,
        print_debug: bool = False,
        debug_img: CVImage | None = None,
    ) -> None:
        # If the model is not loaded, stop here
        if self.model is None:
            return

        self.yolo_msg.detected = []

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

                    r = YoloObject()
                    r.xmin, r.ymin, r.xmax, r.ymax = map(int, box)
                    r.confidence = float(conf)
                    r.classifier = result.names[int(cls_id)]

                    self.yolo_msg.detected.append(r)

                    if print_debug:
                        # Draw the bounding box on the frame
                        cv.rectangle(
                            debug_img,
                            (r.xmin, r.ymin),
                            (r.xmax, r.ymax),
                            MLObjectProcessor.RECT_COLOR,
                            MLObjectProcessor.RECT_WIDTH,
                        )

                        # Get and draw the class name and confidence
                        cv.putText(
                            debug_img,
                            f"{r.classifier} {r.confidence:.2f}",
                            (r.xmin, r.ymin - 10),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            MLObjectProcessor.TEXT_COLOR,
                            MLObjectProcessor.TEXT_THICK,
                        )
        self.yolo_pub.publish(self.yolo_msg)
