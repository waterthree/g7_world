import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
import time

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state = "move_forward"
        self.turn_direction = "right"
        self.largest_area = 0
        self.start_time = self.get_clock().now()
        self.duration_move = 2.0
        self.duration_turn = math.pi / 2 / 0.2
        self.timer = self.create_timer(0.1, self.update_state)

        self.red_threshold = 140000
        self.brown_threshold = 240000
        self.sum_threshold = 220000

        self.turning_start_time = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def update_state(self):
        msg = Twist()

        if self.state == "move_forward":
            msg.linear.x = 0.4
            msg.angular.z = 0.0
            self.turning_start_time = None
        elif self.state == "turn_right":
            msg.linear.x = 0.0
            msg.angular.z = -0.4
        elif self.state == "turn_left":
            msg.linear.x = 0.0
            msg.angular.z = 0.4

        self.publisher_.publish(msg)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: ' + str(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        red_areas = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)

        lower_brown = np.array([10, 100, 20])
        upper_brown = np.array([20, 255, 200])

        mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)

        brown_areas = cv2.bitwise_and(cv_image, cv_image, mask=mask_brown)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_brown, _ = cv2.findContours(mask_brown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area_red = max(cv2.contourArea(contour) for contour in contours_red) if contours_red else 0
        largest_area_brown = max(cv2.contourArea(contour) for contour in contours_brown) if contours_brown else 0

        height, width, _ = cv_image.shape

        if largest_area_red > self.red_threshold or largest_area_brown > self.brown_threshold or (largest_area_red+largest_area_brown) > self.sum_threshold:
            moments = cv2.moments(largest_area_red)
            if moments["m00"] != 0:
                cX = int(moments["m10"] / moments["m00"])
                cY = int(moments["m01"] / moments["m00"])
            else:
                cX, cY = 0, 0
            # Compare the x-coordinate of the centroid with the center of the image
            if cX > width / 2:
                self.state = "turn_left"
            else:
                self.state = "turn_right"
        elif largest_area_red < 50000 and largest_area_brown < 100000 and self.state in ["turn_right", "turn_left"]:
            self.state = "move_forward"
        elif contours_brown and self.state == "move_forward":
            largest_contour_brown = max(contours_brown, key=cv2.contourArea)
            epsilon = 0.02 * cv2.arcLength(largest_contour_brown, True)
            approx = cv2.approxPolyDP(largest_contour_brown, epsilon, True)

            if len(approx) == 6 and largest_area_brown > 70000:
                self.state = "turn_right" if self.turn_direction == "right" else "turn_left"
            else:
                self.state = "move_forward"

        new_width = int(width * 0.5)
        new_height = int(height * 0.5)

        cv_image = cv2.resize(cv_image, (new_width, new_height))
        red_areas = cv2.resize(red_areas, (new_width, new_height))
        brown_areas = cv2.resize(brown_areas, (new_width, new_height))

        dual_images = cv2.hconcat([cv_image, red_areas, brown_areas])

        cv2.putText(dual_images, f'State: {self.state}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(dual_images, f'Largest red area: {largest_area_red}/{self.red_threshold}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(dual_images, f'Largest brown area: {largest_area_brown}/{self.brown_threshold}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(dual_images, f'Sum area: {largest_area_red + largest_area_brown}/{self.sum_threshold}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow("Camera Image", dual_images)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
