import os
from typing import Dict, Iterator, List, Tuple

import cv2 as cv
from ament_index_python import get_package_share_directory
from raubase_msgs.msg import ResultYolo, ObjectYolo
from raubase_ros.interface import CVImage, ImageProcessingUnit
from raubase_ros.utils import DimType
from raubase_ros.wrappers import NodeWrapper, ParameterWrapper
from ultralytics import YOLO
from ultralytics.engine.results import Results

YOLOResults = List[Results]


class MLObjectProcessor(ImageProcessingUnit):
    """
    Detect objects with trained AI (using the YOLO library).
    """

    # ROS things
    YOLO_OUTPUT_TOPIC = "sensor/yolo"
    QOS = 10

    # Debug configuration
    RECT_COLOR = (0, 255, 0)
    RECT_WIDTH = 2

    TEXT_COLOR = (255, 255, 255)
    TEXT_THICK = 2

    # =================================================================
    #                          Initializations
    # =================================================================

    def __init__(self):
        super().__init__("YOLOProcessor")

    def load_model(self, model_name: str) -> YOLO | None:
        # Get model file
        share_dir = get_package_share_directory("dtu_robocup_24")
        file_path = os.path.join(share_dir, "model", f"{model_name}.pt")

        if os.path.exists(file_path):
            self._logger.info(f"Loading model {model_name}")
            return YOLO(file_path)
        else:
            self._logger.error(f"Model {model_name} not found !")
            return None

    def setup(self, node: NodeWrapper) -> None:
        # Load model
        model_name = node.declare_wparameter("yolo_model", "ball_detection").get()
        self.model: YOLO | None = self.load_model(model_name)

        if self.model is None:
            return

        # If model loaded, set an output topic
        self.yolo_msg = ResultYolo()
        self.yolo_msg.detected = []
        self.yolo_pub = node.create_publisher(
            ResultYolo,
            MLObjectProcessor.YOLO_OUTPUT_TOPIC,
            10,
        )

        # Load classes' width
        self.widths: Dict[str, ParameterWrapper] = {
            "ball": node.declare_wparameter("ball_width", 0.04),
            "house": node.declare_wparameter("house_width", 0.1),
            "trolley": node.declare_wparameter("trolley_width", 0.06),
        }
        self.min_conf = node.declare_wparameter("min_conf", 0.6)

    # =================================================================
    #                           Methods
    # =================================================================
    @staticmethod
    def box_results(
        results: YOLOResults,
    ) -> Iterator[Tuple[Tuple[float, float, float, float], float, str]]:
        """
        Iterate over the results of the model
        """
        for r in results:
            for box, conf, c_id in zip(r.boxes.xyxy, r.boxes.conf, r.boxes.cls):
                yield (box, float(conf), r.names[int(c_id)])

    def make_yolo_obj(
        self, boxes: Tuple[float, float, float, float], conf: float, class_name: str
    ) -> ObjectYolo:
        """
        Construct the YOLO object based on the given box.
        """
        r = ObjectYolo()

        # Make image position
        r.xmin, r.ymin, r.xmax, r.ymax = map(int, boxes)
        r.confidence = conf
        r.classifier = class_name

        if r.classifier in self.widths.keys():
            real_width = self.widths.get(r.classifier).get()
        else:
            real_width = 0.1

        # Compute coordinates in robot frame
        width, height = (r.xmax - r.xmin), (r.ymax - r.ymin)
        xc, yc = r.xmin + width / 2, r.ymin + height / 2
        r.robot_x.x, r.robot_x.y, r.robot_x.z = self.in_robot_frame(
            xc, yc, width, real_width, dim_type=DimType.WIDTH
        )
        return r

    @staticmethod
    def draw_debug(debug_img: CVImage, obj: ObjectYolo) -> None:
        """
        Draw the object onto the debug image.
        """
        # Draw the bounding box on the frame
        cv.rectangle(
            debug_img,
            (obj.xmin, obj.ymin),
            (obj.xmax, obj.ymax),
            MLObjectProcessor.RECT_COLOR,
            MLObjectProcessor.RECT_WIDTH,
        )

        # Get and draw the class name and confidence
        cv.putText(
            debug_img,
            f"{obj.classifier} {obj.confidence:.2f}",
            (obj.xmin, obj.ymin - 10),
            cv.FONT_HERSHEY_SIMPLEX,
            0.5,
            MLObjectProcessor.TEXT_COLOR,
            MLObjectProcessor.TEXT_THICK,
        )

    # =================================================================
    #                           Loop
    # =================================================================
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
        results: YOLOResults = self.model([rgb_img], stream=True)

        if results is None:
            return

        for box, conf, class_name in MLObjectProcessor.box_results(results):
            if conf > self.min_conf.get():
                obj = self.make_yolo_obj(box, float(conf), class_name)
                self.yolo_msg.detected.append(obj)

                if print_debug:
                    MLObjectProcessor.draw_debug(debug_img, obj)

        self.yolo_pub.publish(self.yolo_msg)
