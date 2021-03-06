#! /usr/bin/env python3

import os
import cv2
import yaml
import rospy
import rospkg
import threading
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.encoding import Config as DynamicReconfigureConfig
from sensor_msgs.msg import Image, JointState
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineInformationInImage, \
    LineSegmentInImage, ObstaclesInImage, ObstacleInImage, ImageWithRegionOfInterest, GoalPartsInImage, PostInImage, \
    GoalInImage, Speak
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, live_classifier, \
    classifier, ball, fcnn_handler, live_fcnn_03, dummy_ballfinder, obstacle, evaluator, yolo_handler
from bitbots_vision.cfg import VisionConfig
from bitbots_msgs.msg import Config

class Vision:
    def __init__(self):
        # type () -> None
        """
        Vision is the main ROS-node for handling all tasks related to image processing.
        Initiating 'bitbots_vision' node.

        :return: None
        """
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_vision')
        rospy.loginfo('Initializing vision...')

        self.bridge = CvBridge()

        self.config = {}

        # the head_joint_states is used by the dynamic field_boundary detector
        self.head_joint_state = None

        self.debug_image_dings = debug.DebugImage()  # Todo: better variable name
        if self.debug_image_dings:
            self.runtime_evaluator = evaluator.RuntimeEvaluator(None)

        # Register publisher of 'vision_config'-messages
        # For changes of topic name: also change topic name in dynamic_color_space.py
        self.pub_config = rospy.Publisher(
            'vision_config',
            Config,
            queue_size=1,
            latch=True)

        # Speak publisher
        self.speak_publisher = rospy.Publisher('/speak', Speak, queue_size=10)
        self._first_callback = True

        # Register VisionConfig server (dynamic reconfigure) and set callback
        #rospy.logerr('start srv')
        srv = Server(VisionConfig, self._dynamic_reconfigure_callback)
        #rospy.logerr('end err')
        rospy.spin()

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limeted to 1. 
        That's, why we drop old images manually.
        """
        # drops old images and cleans up queue
        image_age = rospy.get_rostime() - image_msg.header.stamp 
        if image_age.to_sec() > 1.0:
            self.debug_printer.info('Vision: Dropped Image-message', 'image')
            return

        self.handle_image(image_msg)

    def _head_joint_state_callback(self, headjoint_msg):
        # type: (JointState) -> None
        """
        Sets a new head_joint_state for the field-boundary-module when a new msg is received
        :param headjoint_msg: the current vertical position of the head
        """
        self.field_boundary_detector.set_head_joint_state(headjoint_msg)

    def _speak(self, string, speech_publisher):
        speak_message = Speak()
        speak_message.text = string
        speech_publisher.publish(speak_message)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')


        if self._first_callback:
            mean = cv2.mean(image)

            if sum(mean) < self._blind_threshold:
                self._speak("Hey!   Remove my camera cap!", self.speak_publisher)

        # setup detectors
        self.field_boundary_detector.set_image(image)
        self.obstacle_detector.set_image(image)
        self.line_detector.set_image(image)

        self.runtime_evaluator.set_image()

        self.ball_detector.set_image(image)

        if self.config['vision_parallelize']:
            self.field_boundary_detector.compute_all()  # computes stuff which is needed later in the processing
            fcnn_thread = threading.Thread(target=self.ball_detector.compute_top_candidate)
            conventional_thread = threading.Thread(target=self._conventional_precalculation())

            conventional_thread.start()
            fcnn_thread.start()

            conventional_thread.join()
            fcnn_thread.join()
        else:
            self.ball_detector.compute_top_candidate()
            self._conventional_precalculation()

        # TODO: handle all ball candidates

        #"""
        ball_candidates = self.ball_detector.get_candidates()

        if ball_candidates:
            balls_under_field_boundary = self.field_boundary_detector.balls_under_convex_field_boundary(ball_candidates)
            if balls_under_field_boundary:
                sorted_rated_candidates = sorted(balls_under_field_boundary, key=lambda x: x.rating)
                top_ball_candidate = list([max(sorted_rated_candidates[0:1], key=lambda x: x.rating)])[0]
            else:
                top_ball_candidate = None
        else:
            top_ball_candidate = None
        """
        # check whether ball candidates are under the field_boundary
        # TODO: handle multiple ball candidates
        top_ball_candidate = self.ball_detector.get_top_candidate()
        if top_ball_candidate:
            ball = []
            ball.append(top_ball_candidate)
            ball_under_field_boundary = self.field_boundary_detector.balls_under_field_boundary(ball)
            if ball_under_field_boundary:
                top_ball_candidate = ball_under_field_boundary[0]
            else:
                top_ball_candidate = None
        #"""

        # check whether ball candidates are over rating threshold
        if top_ball_candidate and top_ball_candidate.rating > self._ball_candidate_threshold:
            # create ball msg
            # TODO: publish empty msg if no top candidate as described in msg description
            balls_msg = BallsInImage()
            balls_msg.header.frame_id = image_msg.header.frame_id
            balls_msg.header.stamp = image_msg.header.stamp

            ball_msg = BallInImage()
            ball_msg.center.x = top_ball_candidate.get_center_x()
            ball_msg.center.y = top_ball_candidate.get_center_y()
            ball_msg.diameter = top_ball_candidate.get_diameter()
            ball_msg.confidence = 1

            balls_msg.candidates.append(ball_msg)
            self.debug_printer.info('found a ball! \o/', 'ball')
            self.pub_balls.publish(balls_msg)

        # create goalpost msg
        goal_parts_msg = GoalPartsInImage()
        goal_parts_msg.header.frame_id = image_msg.header.frame_id
        goal_parts_msg.header.stamp = image_msg.header.stamp

        # create obstacle msg
        obstacles_msg = ObstaclesInImage()
        obstacles_msg.header.frame_id = image_msg.header.frame_id
        obstacles_msg.header.stamp = image_msg.header.stamp
        for red_obs in self.obstacle_detector.get_red_obstacles():
            obstacle_msg = ObstacleInImage()
            obstacle_msg.color = ObstacleInImage.ROBOT_MAGENTA
            obstacle_msg.top_left.x = red_obs.get_upper_left_x()
            obstacle_msg.top_left.y = red_obs.get_upper_left_y()
            obstacle_msg.height = int(red_obs.get_height())
            obstacle_msg.width = int(red_obs.get_width())
            obstacle_msg.confidence = 1.0
            obstacle_msg.playerNumber = 42
            obstacles_msg.obstacles.append(obstacle_msg)
        for blue_obs in self.obstacle_detector.get_blue_obstacles():
            obstacle_msg = ObstacleInImage()
            obstacle_msg.color = ObstacleInImage.ROBOT_CYAN
            obstacle_msg.top_left.x = blue_obs.get_upper_left_x()
            obstacle_msg.top_left.y = blue_obs.get_upper_left_y()
            obstacle_msg.height = int(blue_obs.get_height())
            obstacle_msg.width = int(blue_obs.get_width())
            obstacle_msg.confidence = 1.0
            obstacle_msg.playerNumber = 42
            obstacles_msg.obstacles.append(obstacle_msg)

        if self.config['vision_ball_classifier'] == 'yolo':
            candidates = self.goalpost_detector.get_candidates()
            for goalpost in candidates:
                post_msg = PostInImage()
                post_msg.width = goalpost.get_width()
                post_msg.confidence = goalpost.get_rating()
                post_msg.foot_point.x = goalpost.get_center_x()
                post_msg.foot_point.y = goalpost.get_lower_right_y()
                post_msg.top_point = post_msg.foot_point
                goal_parts_msg.posts.append(post_msg)
        else:
            for white_obs in self.obstacle_detector.get_white_obstacles():
                post_msg = PostInImage()
                post_msg.width = white_obs.get_width()
                post_msg.confidence = 1.0
                post_msg.foot_point.x = white_obs.get_center_x()
                post_msg.foot_point.y = white_obs.get_lower_right_y()
                post_msg.top_point = post_msg.foot_point
                goal_parts_msg.posts.append(post_msg)
        for other_obs in self.obstacle_detector.get_other_obstacles():
            obstacle_msg = ObstacleInImage()
            obstacle_msg.color = ObstacleInImage.UNDEFINED
            obstacle_msg.top_left.x = other_obs.get_upper_left_x()
            obstacle_msg.top_left.y = other_obs.get_upper_left_y()
            obstacle_msg.height = int(other_obs.get_height())
            obstacle_msg.width = int(other_obs.get_width())
            obstacle_msg.confidence = 1.0
            obstacles_msg.obstacles.append(obstacle_msg)
        self.pub_obstacle.publish(obstacles_msg)

        goal_msg = GoalInImage()
        goal_msg.header = goal_parts_msg.header
        left_post = PostInImage()
        left_post.foot_point.x = 9999999999
        left_post.confidence = 1.0
        right_post = PostInImage()
        right_post.foot_point.x = -9999999999
        right_post.confidence = 1.0
        for post in goal_parts_msg.posts:
            if post.foot_point.x < left_post.foot_point.x:
                left_post = post
                left_post.confidence = post.confidence
            if post.foot_point.x > right_post.foot_point.x:
                right_post = post
                right_post.confidence = post.confidence
        goal_msg.left_post = left_post
        goal_msg.right_post = right_post
        goal_msg.confidence = 1.0
        if goal_parts_msg.posts:
            self.pub_goal.publish(goal_msg)

        # create line msg
        line_msg = LineInformationInImage()  # Todo: add lines
        line_msg.header.frame_id = image_msg.header.frame_id
        line_msg.header.stamp = image_msg.header.stamp
        for lp in self.line_detector.get_linepoints():
            ls = LineSegmentInImage()
            ls.start.x = lp[0]
            ls.start.y = lp[1]
            ls.end = ls.start
            line_msg.segments.append(ls)
        self.pub_lines.publish(line_msg)

        # create non_line msg
        # non_line_msg = LineInformationInImage()
        # non_line_msg.header.frame_id = image_msg.header.frame_id
        # non_line_msg.header.stamp = image_msg.header.stamp
        # i = 0
        # for nlp in self.line_detector.get_nonlinepoints():
        #     nls = LineSegmentInImage()
        #     nls.start.x = nlp[0]
        #     nls.start.y = nlp[1]
        #     nls.end = nls.start
        #     if i % 2 == 0:
        #         non_line_msg.segments.append(nls)
        #     i += 1
        # self.pub_non_lines.publish(non_line_msg)

        if self.ball_fcnn_publish_output and self.config['vision_ball_classifier'] == 'fcnn':
            self.pub_ball_fcnn.publish(self.ball_detector.get_cropped_msg())

        if self.publish_fcnn_debug_image and self.config['vision_ball_classifier'] == 'fcnn':
            self.pub_debug_fcnn_image.publish(self.ball_detector.get_debug_image())

        # do debug stuff
        if self.publish_debug_image:
            self.debug_image_dings.set_image(image)
            self.debug_image_dings.draw_obstacle_candidates(
                self.obstacle_detector.get_candidates(),
                (0, 0, 0),
                thickness=3
            )
            self.debug_image_dings.draw_obstacle_candidates(
                self.obstacle_detector.get_red_obstacles(),
                (0, 0, 255),
                thickness=3
            )
            self.debug_image_dings.draw_obstacle_candidates(
                self.obstacle_detector.get_blue_obstacles(),
                (255, 0, 0),
                thickness=3
            )
            if self.config['vision_ball_classifier'] == "yolo":
                post_candidates = self.goalpost_detector.get_candidates()
            else:
                post_candidates = self.obstacle_detector.get_white_obstacles()
            self.debug_image_dings.draw_obstacle_candidates(
                post_candidates,
                (255, 255, 255),
                thickness=3
            )
            self.debug_image_dings.draw_field_boundary(
                self.field_boundary_detector.get_field_boundary_points(),
                (0, 0, 255))
            self.debug_image_dings.draw_field_boundary(
                self.field_boundary_detector.get_convex_field_boundary_points(),
                (0, 255, 255))
            self.debug_image_dings.draw_ball_candidates(
                self.ball_detector.get_candidates(),
                (0, 0, 255))
            self.debug_image_dings.draw_ball_candidates(
                self.field_boundary_detector.balls_under_field_boundary(
                    self.ball_detector.get_candidates(),
                    self._ball_candidate_y_offset),
                (0, 255, 255))
            # draw top candidate in
            self.debug_image_dings.draw_ball_candidates([top_ball_candidate],
                                                        (0, 255, 0))
            # draw linepoints in red
            self.debug_image_dings.draw_points(
                self.line_detector.get_linepoints(),
                (0, 0, 255))
            # draw nonlinepoints in black
            # self.debug_image_dings.draw_points(
            #     self.line_detector.get_nonlinepoints(),
            #     (0, 0, 0))

            # publish debug image
            self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(self.debug_image_dings.get_image(), 'bgr8'))

        self._first_callback = False

    def _conventional_precalculation(self):
        self.obstacle_detector.compute_all_obstacles()
        self.line_detector.compute_linepoints()

    def _dynamic_reconfigure_callback(self, config, level):
        #rospy.logerr("in dynamic re callback")
        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                config['vision_debug_printer_classes']))
        self.runtime_evaluator = evaluator.RuntimeEvaluator(self.debug_printer)

        self._blind_threshold = config['vision_blind_threshold']
        self._ball_candidate_threshold = config['vision_ball_candidate_rating_threshold']
        self._ball_candidate_y_offset = config['vision_ball_candidate_field_boundary_y_offset']

        self.publish_debug_image = config['vision_publish_debug_image']
        if self.publish_debug_image:
            rospy.logwarn('Debug images are enabled')
        else:
            rospy.loginfo('Debug images are disabled')
        self.ball_fcnn_publish_output = config['ball_fcnn_publish_output']
        if self.ball_fcnn_publish_output:
            rospy.logwarn('ball FCNN output publishing is enabled')
        else:
            rospy.logwarn('ball FCNN output publishing is disabled')

        self.publish_fcnn_debug_image = config['ball_fcnn_publish_debug_img']

        if config['vision_ball_classifier'] == 'dummy':
            self.ball_detector = dummy_ballfinder.DummyClassifier(None, None, self.debug_printer)

        # Print status of color config
        if 'vision_use_sim_color' not in self.config or \
            config['vision_use_sim_color'] != self.config['vision_use_sim_color']:
            if config['vision_use_sim_color']:
                rospy.logwarn('Loaded color space for SIMULATOR.')
            else:
                rospy.loginfo('Loaded color space for REAL WORLD.')

        self.white_color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            [config['white_color_detector_lower_values_h'], config['white_color_detector_lower_values_s'],
             config['white_color_detector_lower_values_v']],
            [config['white_color_detector_upper_values_h'], config['white_color_detector_upper_values_s'],
             config['white_color_detector_upper_values_v']])

        self.red_color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            [config['red_color_detector_lower_values_h'], config['red_color_detector_lower_values_s'],
             config['red_color_detector_lower_values_v']],
            [config['red_color_detector_upper_values_h'], config['red_color_detector_upper_values_s'],
             config['red_color_detector_upper_values_v']])

        self.blue_color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            [config['blue_color_detector_lower_values_h'], config['blue_color_detector_lower_values_s'],
             config['blue_color_detector_lower_values_v']],
            [config['blue_color_detector_upper_values_h'], config['blue_color_detector_upper_values_s'],
             config['blue_color_detector_upper_values_v']])

        if config['dynamic_color_space_active']:
            self.field_color_detector = color.DynamicPixelListColorDetector(
                self.debug_printer,
                self.package_path,
                config,
                primary_detector=True)
        else:
            self.field_color_detector = color.PixelListColorDetector(
                self.debug_printer,
                self.package_path,
                config)

        self.field_boundary_detector = field_boundary.FieldBoundaryDetector(
            self.field_color_detector,
            config,
            self.debug_printer,
            self.runtime_evaluator)

        self.line_detector = lines.LineDetector(
            self.white_color_detector,
            self.field_color_detector,
            self.field_boundary_detector,
            config,
            self.debug_printer)

        self.obstacle_detector = obstacle.ObstacleDetector(
            self.red_color_detector,
            self.blue_color_detector,
            self.white_color_detector,
            self.field_boundary_detector,
            self.runtime_evaluator,
            config,
            self.debug_printer
        )

        # set up ball config for fcnn
        # these config params have domain-specific names which could be problematic for fcnn handlers handling e.g. goal candidates
        # this enables 2 fcnns with different configs.
        self.ball_fcnn_config = {
            'debug': config['ball_fcnn_publish_debug_img'],
            'threshold': config['ball_fcnn_threshold'],
            'expand_stepsize': config['ball_fcnn_expand_stepsize'],
            'pointcloud_stepsize': config['ball_fcnn_pointcloud_stepsize'],
            'shuffle_candidate_list': config['ball_fcnn_shuffle_candidate_list'],
            'min_candidate_diameter': config['ball_fcnn_min_ball_diameter'],
            'max_candidate_diameter': config['ball_fcnn_max_ball_diameter'],
            'candidate_refinement_iteration_count': config['ball_fcnn_candidate_refinement_iteration_count'],
            'publish_field_boundary_offset': config['ball_fcnn_publish_field_boundary_offset'],
        }

        # load fcnn
        if config['vision_ball_classifier'] == 'fcnn':
            if 'neural_network_model_path' not in self.config or \
                    self.config['neural_network_model_path'] != config['neural_network_model_path'] or \
                    self.config['vision_ball_classifier'] != config['vision_ball_classifier']:
                ball_fcnn_path = os.path.join(self.package_path, 'models', config['neural_network_model_path'])
                if not os.path.exists(ball_fcnn_path):
                    rospy.logerr('AAAAHHHH! The specified fcnn model file doesn\'t exist!')
                self.ball_fcnn = live_fcnn_03.FCNN03(ball_fcnn_path, self.debug_printer)
                rospy.loginfo(config['vision_ball_classifier'] + " vision is running now")
            self.ball_detector = fcnn_handler.FcnnHandler(
                self.ball_fcnn,
                self.field_boundary_detector,
                self.ball_fcnn_config,
                self.debug_printer)

        
        if config['vision_ball_classifier'] == 'yolo':
            if 'neural_network_model_path' not in self.config or \
                    self.config['neural_network_model_path'] != config['neural_network_model_path'] or \
                    self.config['vision_ball_classifier'] != config['vision_ball_classifier']:
                yolo_model_path = os.path.join(self.package_path, 'models', config['neural_network_model_path'])
                if not os.path.exists(yolo_model_path):
                    rospy.logerr('AAAAHHHH! The specified yolo model file doesn\'t exist!')
                # TODO replace following strings with path to config/weights
                yolo = yolo_handler.YoloHandler(config, yolo_model_path)
                self.ball_detector = yolo_handler.YoloBallDetector(yolo)
                self.goalpost_detector = yolo_handler.YoloGoalpostDetector(yolo)
                rospy.loginfo(config['vision_ball_classifier'] + " vision is running now")
         
        # publishers

        # TODO: topic: ball_in_... BUT MSG TYPE: balls_in_img... CHANGE TOPIC TYPE!
        if 'ROS_ball_msg_topic' not in self.config or \
                self.config['ROS_ball_msg_topic'] != config['ROS_ball_msg_topic']:
            if hasattr(self, 'pub_balls'):
                self.pub_balls.unregister()
            self.pub_balls = rospy.Publisher(
                config['ROS_ball_msg_topic'],
                BallsInImage,
                queue_size=1)

        if 'ROS_line_msg_topic' not in self.config or \
                self.config['ROS_line_msg_topic'] != config['ROS_line_msg_topic']:
            if hasattr(self, 'pub_lines'):
                self.pub_lines.unregister()
            self.pub_lines = rospy.Publisher(
                config['ROS_line_msg_topic'],
                LineInformationInImage,
                queue_size=5)

        # publisher for nonlinepoints
        # if 'ROS_non_line_msg_topic' not in self.config or \
        #         self.config['ROS_non_line_msg_topic'] != config['ROS_non_line_msg_topic']:
        #     if hasattr(self, 'pub_non_lines'):
        #         self.pub_non_lines.unregister()
        #     self.pub_non_lines = rospy.Publisher(
        #         config['ROS_non_line_msg_topic'],
        #         LineInformationInImage,
        #         queue_size=5)

        if 'ROS_obstacle_msg_topic' not in self.config or \
                self.config['ROS_obstacle_msg_topic'] != config['ROS_obstacle_msg_topic']:
            if hasattr(self, 'pub_obstacle'):
                self.pub_obstacle.unregister()
            self.pub_obstacle = rospy.Publisher(
                config['ROS_obstacle_msg_topic'],
                ObstaclesInImage,
                queue_size=3)

        if 'ROS_goal_msg_topic' not in self.config or \
                self.config['ROS_goal_msg_topic'] != config['ROS_goal_msg_topic']:
            if hasattr(self, 'pub_goal'):
                self.pub_goal.unregister()
            self.pub_goal = rospy.Publisher(
                config['ROS_goal_msg_topic'],
                GoalInImage,
                queue_size=3)

        if 'ROS_fcnn_img_msg_topic' not in self.config or \
                self.config['ROS_fcnn_img_msg_topic'] != config['ROS_fcnn_img_msg_topic']:
            if hasattr(self, 'pub_ball_fcnn'):
                self.pub_ball_fcnn.unregister()
            self.pub_ball_fcnn = rospy.Publisher(
                config['ROS_fcnn_img_msg_topic'],
                ImageWithRegionOfInterest,
                queue_size=1)

        if 'ROS_debug_image_msg_topic' not in self.config or \
                self.config['ROS_debug_image_msg_topic'] != config['ROS_debug_image_msg_topic']:
            if hasattr(self, 'pub_debug_image'):
                self.pub_debug_image.unregister()
            self.pub_debug_image = rospy.Publisher(
                config['ROS_debug_image_msg_topic'],
                Image,
                queue_size=1)

        if 'ROS_debug_fcnn_image_msg_topic' not in self.config or \
                self.config['ROS_debug_fcnn_image_msg_topic'] != config['ROS_debug_fcnn_image_msg_topic']:
            if hasattr(self, 'pub_debug_fcnn_image'):
                self.pub_debug_fcnn_image.unregister()
            self.pub_debug_fcnn_image = rospy.Publisher(
                config['ROS_debug_fcnn_image_msg_topic'],
                Image,
                queue_size=1)

        # subscribers
        if 'ROS_img_msg_topic' not in self.config or \
                self.config['ROS_img_msg_topic'] != config['ROS_img_msg_topic']:
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

        # subscriber for the vertical position of the head, used by the dynamic field-boundary-detector
        if 'ROS_head_joint_msg_topic' not in self.config or \
                self.config['ROS_head_joint_msg_topic'] != config['ROS_head_joint_msg_topic']:
            if hasattr(self, 'head_sub'):
                self.head_sub.unregister()
            self.head_sub = rospy.Subscriber(
                config['ROS_head_joint_msg_topic'],
                JointState,
                self._head_joint_state_callback,
                queue_size=config['ROS_head_joint_state_queue_size'],
                tcp_nodelay=True)

        # Publish Config-message
        # Clean config dict to avoid not dumpable types
        config_cleaned = {}
        for key, value in config.items():
            if type(value) != DynamicReconfigureConfig:
                config_cleaned[key] = value
        msg = Config()
        msg.data = yaml.dump(config_cleaned)
        self.pub_config.publish(msg)

        self.config = config
        return config

if __name__ == '__main__':
    myVision = Vision()
  
