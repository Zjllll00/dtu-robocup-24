import cv2 as cv
import cv2.aruco as ArUco
from raubase_ros.interface import CVImage, ImageProcessingUnit
from raubase_ros.wrappers import NodeWrapper, ParameterWrapper
from raubase_msgs.msg import ObjectArUco, ResultArUco
from typing import Dict, Sequence
import numpy as np
from scipy.spatial.transform import Rotation

MatLike = cv.typing.MatLike


class ArUcoProcessor(ImageProcessingUnit):
    """
    Detect ArUco markers in OpenCV Images.
    """

    ARUCO_TOPIC = "aruco"
    DEF_CODE_WIDTH = 0.08

    # =================================================================
    #                          Initializations
    # =================================================================
    def __init__(self, aruco_dict: int = ArUco.DICT_4X4_250):
        super().__init__("ArUcoProcessor")
        self.__detector = ArUco.ArucoDetector(
            ArUco.getPredefinedDictionary(aruco_dict),
            ArUco.DetectorParameters(),
        )

    def setup(self, node: NodeWrapper) -> None:
        self.aruco_msg = ResultArUco()
        self.aruco_pub = node.create_publisher(
            ResultArUco, ArUcoProcessor.ARUCO_TOPIC, 10
        )

        numbers = node.declare_wparameter("aurco_codes", [0, 12, 3, 10]).get()
        self.code_width: Dict[int, ParameterWrapper[float]] = {}
        self.code_3d_points: Dict[str, np.ndarray] = {}
        for n in numbers:
            self.code_width[n] = node.declare_wparameter(
                f"aruco_{n}_m", ArUcoProcessor.DEF_CODE_WIDTH
            )
            self.make_marker_3D_points(self.code_width[n].get())

    def make_marker_3D_points(self, width: float):
        """
        Making the 3D pointers associated to the corners of the ArUco code.
        """
        k = f"{width:.3f}"

        # If 3D points already generated, skip it
        if k in self.code_3d_points.keys():
            return

        # Else make it
        self.code_3d_points[k] = np.array(
            [
                [-width / 2, width / 2, 0],
                [width / 2, width / 2, 0],
                [width / 2, -width / 2, 0],
                [-width / 2, -width / 2, 0],
            ],
            dtype=np.float32,
        )

    def get_marker_3D_points(self, width: float) -> np.ndarray:
        """
        Get the 3D points corresponding to the corners of the ArUco code.
        """
        k = f"{width:.3f}"

        if k not in self.code_3d_points.keys():
            self.make_marker_3D_points(width)

        return self.code_3d_points[k]

    def get_marker_3D_points_N(self, id: int) -> np.ndarray:
        if id in self.code_width.keys():
            return self.get_marker_3D_points(self.code_width[id].get())
        return self.get_marker_3D_points(ArUcoProcessor.DEF_CODE_WIDTH)

    # =================================================================
    #                           Methods
    # =================================================================
    def make_aruco_obj(self, cnrs: MatLike, id: int) -> ObjectArUco:
        """
        Construct an ArUco code result object.
        """
        r = ObjectArUco()
        r.id = int(id)

        # Add corners
        r.corners_x.resize((4))
        r.corners_y.resize((4))
        for i in range(4):
            r.corners_x[i] = float(cnrs[i, 0])
            r.corners_y[i] = float(cnrs[i, 1])

        # Try to get the position of the ArUco codes
        success, rot, t = cv.solvePnP(
            self.get_marker_3D_points_N(r.id),
            cnrs,
            self.data.cam_info.k.reshape((3, 3)),
            np.array(self.data.cam_info.d),
            flags=cv.SOLVEPNP_ITERATIVE,
        )
        if not success:
            r.x.x, r.x.y, r.x.z = t[0], t[1], t[2]
            rot_mat = Rotation.from_rotvec([rot[0], rot[1], rot[2]]).as_matrix()
            r.rx = rot_mat[:, 0]
            r.ry = rot_mat[:, 1]
            r.rz = rot_mat[:, 2]
        else:
            self._logger.warn("Failed to compute position and frame for ArUco code!")

        return r

    @staticmethod
    def draw_debug(
        debug_img: CVImage,
        cnrs: Sequence[MatLike],
        ids: MatLike,
        rjcts: Sequence[MatLike],
    ):
        """
        Draw the objects onto the debug image.
        """
        # Draw detected
        if ids is not None:
            ArUco.drawDetectedMarkers(debug_img, cnrs, ids)
        ArUco.drawDetectedMarkers(debug_img, rjcts, None, (255, 255, 20))

    # =================================================================
    #                           Loop
    # =================================================================
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

        # Make ArUco objects
        if ids is not None:
            for corners, id in zip(cnrs, ids):
                self.aruco_msg.detected.append(self.make_aruco_obj(corners[0], int(id)))

        self.aruco_pub.publish(self.aruco_msg)

        # DEBUG: draw them
        if print_debug:
            ArUcoProcessor.draw_debug(debug_img, cnrs, ids, rjcts)
