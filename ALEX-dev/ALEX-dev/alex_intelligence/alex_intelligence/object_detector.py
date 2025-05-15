import cv2
import logging
import numpy as np

from ultralytics import YOLO

logger = logging.getLogger(__name__)

PointXYZ = tuple[float, float, float]

class ObjectDetector3D:
    """
    model weights:  https://neuralfoundry.sharepoint.com/:f:/s/NF-AI/EiH-g-1DI9VOqHd6iviJShIBk8B-W2a4rOe4wUbhO9pgUw?e=DsMIlJ
    """
    def __init__(self, model_path: str, camera_params: dict={}, min_confidence=0.3, debug=False):
        """
        Initialize the camera parameters and model.
        """
        self.debug = debug
        self.model = YOLO(model_path)
        self.min_confidence = min_confidence
        self.classes = ['band', 'beaker', 'conical_flask', 'volumetric_flask']
        self.band_class = self.classes.index('band') or 0
        
        # Set camera parameters from dictionary, with fallback defaults
        self.fx = camera_params.get('fx', 250.20)  # Focal length x
        self.fy = camera_params.get('fy', 250.20)  # Focal length y
        self.ppx = camera_params.get('ppx', 160.0)  # Principal point x
        self.ppy = camera_params.get('ppy', 120.0)  # Principal point y    

    def _calculate_depth_at_mid_point(self, bbox: np.ndarray, depth_image: np.ndarray) -> PointXYZ:
        """
        Returns Pixel_x, Pixel_y, Depth_value as output (tuple)
        """
        x1, y1, x2, y2 = bbox
        center_x = int((x1 + x2) // 2)
        center_y = int((y1 + y2) // 2)
        depth_value_mid_point = depth_image[center_y, center_x]  # Note: (y, x) order for accessing depth image
        depth_value = depth_value_mid_point
        return center_x, center_y, depth_value

    def _is_band_inside_target(self, target, band):
        """
        Checks if the center point of bounding box band is inside the bounds of bounding box target.

        Args:
            target: numpy array representing [x_min, y_min, x_max, y_max] of box target.
            band: numpy array representing [x_min, y_min, x_max, y_max] of box band.

        Returns:
            bool: True if the center of band is inside target, False otherwise.
        """
        center_x = (band[0] + band[2]) / 2
        center_y = (band[1] + band[3]) / 2
        # Check if the center of band is within the bounds of target
        is_inside = (target[0] <= center_x <= target[2]) and (target[1] <= center_y <= target[3])
        return is_inside

    def get_bounding_box(self, img: np.ndarray, target_class=0) -> np.ndarray | None:
        results = self.model([img], conf=self.min_confidence)
        if not results or not results[0]:
            return None
        results = results[0]
        if self.debug:
            cv2.imwrite('debug-detector-results.jpg', results.plot())
        bands = [box for box in results if box.boxes.cls == self.band_class]
        targets = [box for box in results if box.boxes.cls == target_class]
        if not targets:
            return None
        if len(targets) > 1:
            # This warning can be useful
            logger.warning(f'Object detector picked up multiple objects of target class ({len(targets)}), selecting index 0, should use ID tag')
        target = targets[0]
        target_bands = [
            band for band in bands 
            if self._is_band_inside_target(target.boxes.data.cpu().numpy()[0], band.boxes.data.cpu().numpy()[0])
            ]
        if len(target_bands) == 0:
            return None
        target_band = target_bands[0][0]
        if self.debug:
            cv2.imwrite('debug-detector-targets.jpg', target.plot())
            cv2.imwrite('debug-detector-band.jpg', target_band.plot())
        return target_band.boxes.xyxy.cpu().numpy()[0]

    def get_3d_coordinates(self, bbox: np.ndarray, depth_frame: np.ndarray) -> PointXYZ:
        """
        Calculates the 3D coordinates from the given bounding box and depth frame.
        Y is inverted due to pinhole camera logic
        https://support.stereolabs.com/hc/en-us/articles/4554115218711-How-can-I-convert-3D-world-coordinates-to-2D-image-coordinates-and-viceversa
        """
        center_x, center_y, depth = self._calculate_depth_at_mid_point(bbox, depth_frame)
        x = (center_x - self.ppx) * depth / self.fx
        y = (center_y - self.ppy) * depth / self.fy
        z = depth
        return x, -y, z

    # TODO make this throw meaningfull exceptions
    def process(self, img_rgb, img_depth, target_class='band') -> PointXYZ | None:
        if target_class not in self.classes:
            logger.error(f'Invalid class name requested {target_class}')
            return None
        bbox = self.get_bounding_box(img_rgb, target_class=self.classes.index(target_class))
        if bbox is None:
            return None
        return self.get_3d_coordinates(bbox, img_depth)

def test():
    # Configured for Zed camera 720p
    rgb_image: np.ndarray = cv2.imread('alex_intelligence/resource/test_lab_rgb.png')
    depth_image: np.ndarray = cv2.imread('alex_intelligence/resource/test_lab_depth.tiff', cv2.IMREAD_UNCHANGED)

    # Define camera parameters as a dictionary
    camera_params = {
        'fx': 733.0479125976562,
        'fy': 733.0479125976562,
        'ppx': 636.56005859375,
        'ppy': 350.62615966796875
    }

    # Instantiate and process
    object_detection = ObjectDetector3D(
        model_path='alex_intelligence/resource/lab_v3.pt',
        camera_params=camera_params
    )
    point = object_detection.process(rgb_image, depth_image, 'volumetric_flask')
    if point:
        print(f'Point of object: {point}')
    else:
        print('No object detected')

if __name__ == "__main__":
    test()