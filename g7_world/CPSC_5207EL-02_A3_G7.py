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
        # Initialize the node with the name 'turtlebot_controller'
        super().__init__('turtlebot_controller')

        # Movement control setup
        # Create a publisher for sending velocity commands
        # This publisher will send messages of type Twist to the 'cmd_vel' topic,
        # which is commonly used for controlling robot motion. The queue size of 10
        # ensures that up to 10 messages can be buffered for sending if necessary,
        # managing the flow of commands under varying system loads.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Initial state for the movement logic
        self.state = "move_forward"
        self.largest_area = 0
        # Record start time for timing the movements
        self.start_time = self.get_clock().now()
        # Define durations for moving forward and turning
        self.duration_move = 2.0  # Move forward for 2 seconds
        self.duration_turn = math.pi / 2 / 0.2  # Time to turn 90 degrees at 0.2 rad/s
        # Create a timer to periodically update the robot's state
        self.timer = self.create_timer(0.1, self.update_state)

        # Subscribe to the camera topic to receive image messages
        # Create a subscription to listen for messages on the '/camera/image_raw' topic,
        # using the Image message type. The 'image_callback' function is called for each new message,
        # with a queue size of 10 to buffer messages if they arrive faster than they can be processed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()  # Initialize a CvBridge to convert ROS images to OpenCV format

    def update_state(self):

        msg = Twist()

        # Set the velocity based on the current state
        if self.state == "move_forward":
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        elif self.state == "turn":
            msg.linear.x = 0.0
            msg.angular.z = -0.2  # Clockwise rotation

        self.publisher_.publish(msg)  # Publish the velocity command

    def image_callback(self, msg):
        # This method is called with each new image message from the camera
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: ' + str(e))
            return

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color range for red in HSV space
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        # Create a mask for the red areas
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        red_areas = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)

        # Define color range for brown in HSV space
        lower_brown = np.array([10, 100, 20])
        upper_brown = np.array([20, 255, 200])

        # Create a mask for the brown areas
        mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)

        # Bitwise-AND mask and original image
        brown_areas = cv2.bitwise_and(cv_image, cv_image, mask=mask_brown)

        # Find contours in the red mask
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find contours in the brown mask
        contours_brown, _ = cv2.findContours(mask_brown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check the area of each contour
        largest_area_red = max(cv2.contourArea(contour) for contour in contours_red) if contours_red else 0
        largest_area_brown = max(cv2.contourArea(contour) for contour in contours_brown) if contours_brown else 0

        # Update the state based on the largest red area and largest brown area
        if largest_area_red > 70000 or largest_area_brown > 100000 or (largest_area_red+largest_area_brown) > 100000:  # Adjust this threshold as needed
            self.state = "turn"
        elif largest_area_red < 25000 and largest_area_brown < 50000 and self.state == "turn":
            time.sleep(0.2)  # Pause for 0.2 seconds
            self.state = "move_forward"

        # Get the original image size
        height, width, _ = cv_image.shape

        # Calculate the new size
        new_width = int(width * 0.5)
        new_height = int(height * 0.5)

        # Resize the images
        cv_image = cv2.resize(cv_image, (new_width, new_height))
        red_areas = cv2.resize(red_areas, (new_width, new_height))
        brown_areas = cv2.resize(brown_areas, (new_width, new_height))

        # Concatenate the original image and the image with red areas highlighted
        dual_images = cv2.hconcat([cv_image, red_areas, brown_areas])

        # Display the state and largest red area on the image
        cv2.putText(dual_images, f'State: {self.state}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(dual_images, f'Largest red area: {largest_area_red}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(dual_images, f'Largest brown area: {largest_area_brown}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(dual_images, f'Sum area: {largest_area_red + largest_area_brown}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Display the OpenCV image in a window
        cv2.imshow("Camera Image", dual_images)
        cv2.waitKey(1)  # Wait a bit for the window to update

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 Python client library
    turtlebot_controller = TurtlebotController()  # Create the Turtlebot controller node
    rclpy.spin(turtlebot_controller)  # Keep the node running and responsive
    # Cleanup before exiting
    turtlebot_controller.destroy_node()
    cv2.destroyAllWindows()  # Close the OpenCV window
    rclpy.shutdown()  # Shutdown ROS2 Python client library

if __name__ == '__main__':
    main()