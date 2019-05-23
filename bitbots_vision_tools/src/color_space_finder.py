#! /usr/bin/env python2

import cv2
import rospy
import rospkg
import os.path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from bitbots_vision.vision_modules import color, debug
from bitbots_vision_tools.cfg import ColorSpaceFinderConfig

# TODO: Vision.py handle not existing color space file
# TODO: Color.py loading color space -> debug printer
# TODO: docs
# TODO: publish mask
# TODO: update dependancies and check rosdep

class ColorSpaceFinder:
    """
    The ColorSpaceFinder is a tool to quickly find the right color detector parameters.
    This tool creates binary masks of the raw image.
    The parameters are dynamically adjustable.

    :return: None
    """
    def __init__(self):
        # type: () -> None
        """
        Initiating 'bitbots_vision_tools' node.
        """
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision_tools')

        # ROS-Stuff:
        rospy.init_node('bitbots_vision_color_test')
        rospy.loginfo('Initializing color space finder...')

        self.bridge = CvBridge()

        self.config = {}

        # Register ColorSpaceFinderConfig server (dynamic reconfigure) and set callback
        srv = Server(ColorSpaceFinderConfig, self._dynamic_reconfigure_callback)

        rospy.loginfo('Finished initializing of color space finder.')
        
        rospy.spin()

    def _dynamic_reconfigure_callback(self, config, level):
        # type: (dict, int) -> dict
        """
        TODO: docs
        """
        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                config['debug_printer_classes']))

        if self.changed_config_param(config, 'color_detector'):
            if config['color_detector'] == 'HsvColorDetector':
                self.color_detector = color.HsvSpaceColorDetector(
                    self.debug_printer,
                    [config['HSV_lower_values_h'],
                    config['HSV_lower_values_s'],
                    config['HSV_lower_values_v']],
                    [config['HSV_upper_values_h'],
                    config['HSV_upper_values_s'],
                    config['HSV_upper_values_v']])
                rospy.loginfo('Loaded HSV color detector.')

            elif self.config['color_detector'] == 'PixelListColorDetector':
                color_detector = color.PixelListColorDetector(
                    self.debug_printer,
                    self.package_path,
                    config)
                rospy.loginfo('Loaded pixel list color detector.')
            
            elif self.config['color_detector'] == 'DynamicPixelListColorDetector':
                color_detector = color.PixelListColorDetector(
                    self.debug_printer,
                    self.package_path,
                    config,
                    primary_detector=True)
                rospy.loginfo('Loaded dynamic pixel list color detector.')

            else:
                rospy.logwarn('Unknown color detector selected!')

        # subscribers
        if self.changed_config_param(config, 'ROS_img_msg_topic'):
            if hasattr(self, 'image_sub'):
                self.image_sub.unregister()
            self.image_sub = rospy.Subscriber(
                config['ROS_img_msg_topic'],
                Image,
                self._image_callback,
                queue_size=config['ROS_img_queue_size'],
                tcp_nodelay=True,
                buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.config = config
        return config

    def _image_callback(self, img):
        # type: (TODO) -> None
        """
        TODO: docs
        """
        self.handle_image(img)

    def handle_image(self, image_msg):
        # type: (TODO) -> None
        """
        TODO: docs
        """
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # mask image
        self.color_detector.mask_image(image)

    def changed_config_param(self, config, param_name):
        # type: (dict, str) -> bool
        """
        TODO
        """
        return param_name not in self.config or config[param_name] != self.config[param_name]

if __name__ == "__main__":
    ColorSpaceFinder()