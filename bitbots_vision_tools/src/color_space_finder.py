#! /usr/bin/env python2

import cv2
import rospy
import rospkg
import os.path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from bitbots_vision.vision_modules import color
from bitbots_vision_tools.cfg import ColorSpaceFinderConfig

# TODO: rename ColorTest to ColorSpaceFinder
# TODO: launch: optional parameter for starting image provider
# TODO: refactor dyn rec callback
# TODO: handle dynamic color detector

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
        self.use_pixel_list = rospy.get_param(
            'color_test_params/color_detector_implementation/use_PixelListColorDetector')
        # default configuration
        if self.use_pixel_list:
            self.color_detector = color.PixelListColorDetector(
                self.package_path +
                rospy.get_param('color_test_params/field_color_detector/path'))
        else:
            self.color_detector = color.HsvSpaceColorDetector(
                rospy.get_param('color_test_params/white_color_detector/lower_values'),
                rospy.get_param('color_test_params/white_color_detector/upper_values'))

        # subscriber:
        rospy.Subscriber(rospy.get_param('color_test_params/ROS/img_msg_topic'),
                         Image,
                         self._image_callback,
                         queue_size=rospy.get_param(
                             'color_test_params/ROS/img_queue_size'))

        # -----------------------------------------------------------------

        self.debug_mode = config['debug_on']
        # exchange color_detector between PixelListColorDetector and HsvSpaceColorDetector
        self.use_pixel_list = config['use_pixel_list']
        if self.debug_mode:
            print('use_pixel_list = ' + str(self.use_pixel_list))
        if self.use_pixel_list:
            # TODO: fix PixelListColorDetector
            # use PixelListColorDetector
            if os.path.isfile(self.package_path +
                              config['pixel_list_path']):
                self.color_detector = color.PixelListColorDetector(self.package_path +
                                                                   config['pixel_list_path'])
                if self.debug_mode:
                    print('color_detector is instance of PixelListColorDetector('
                          + self.package_path
                          + config['pixel_list_path'])
            else:
                print('FILE "'
                      + self.package_path + config['pixel_list_path'] + '" does NOT EXIST')
        else:
            # use HsvSpaceColorDetector
            self.color_detector = color.HsvSpaceColorDetector((config['h_min'], config['s_min'], config['v_min']),
                                                              (config['h_max'], config['s_max'], config['v_max']))
            if self.debug_mode:
                print('color_detector is instance of HsvSpaceColorDetector('
                      + str((config['h_min'], config['s_min'], config['v_min']))
                      + ', '
                      + str((config['h_max'], config['s_max'], config['v_max'])))
        return config


    def _image_callback(self, img):
        self.handle_image(img)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # mask image
        mask_img = self.color_detector.mask_image(image)

        # output masked image
        cv2.imshow('Debug Image', mask_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    ColorSpaceFinder()