import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import cv2
from cv_bridge import CvBridge

import numpy as np
import socket
import threading
import time
from datetime import datetime
import os

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')

        # Constants
        self.recording = False
        self.video_writer = None
        self.current_tree_id = None
        self.cv_image = None
        self.image_lock = threading.Lock()

        # Create directories
        
        os.makedirs('recordings', exist_ok=True)

        # Create bridge
        self.bridge = CvBridge()

        # Socket server setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 12345))
        self.server_socket.listen(1)
        
        # Socket setup with error handling
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.bind(('0.0.0.0', 12345))
            self.server_socket.listen(1)
            
            self.socket_thread = threading.Thread(target=self.handle_socket_connection)
            self.socket_thread.daemon = True
            self.socket_thread.start()
        except Exception as e:
            self.get_logger().error(f"Socket setup failed: {e}")
            raise

        # Subscribers
        self.image_subscriber = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Publishers
        self.servo_publisher = self.create_publisher(Float64MultiArray, "servo_base_controller/commands", 10)

        self.create_timer(0.1, self.center_ffb)

    def image_callback(self, msg):
        try:
            with self.image_lock:
                self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error Converting: {e}")

    def detect_ffb(self, frame):
        """Detect FFB using color segmentation"""
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define FFB color range (adjust these values based on your FFB color)
            lower_ffb = np.array([0, 100, 100])   # Example values for reddish color
            upper_ffb = np.array([10, 255, 255])
            
            mask = cv2.inRange(hsv, lower_ffb, upper_ffb)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum area threshold
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        return True, (cx, cy), area
            
            return False, None, 0
            
        except Exception as e:
            self.get_logger().error(f'FFB detection error: {str(e)}')
            return False, None, 0

    def center_ffb(self):
        """Center FFB in frame"""
        if self.cv_image is None:
            self.get_logger().warn('No image available for FFB detection')
            return False
        
        """Center FFB in frame at start of recording"""
        self.get_logger().info('Attempting to center FFB...')
        frame_height = 1920
        frame_center_y = frame_height // 2
        threshold = 5
        attempts = 10
        
        # Collect multiple FFB positions to average
        ffb_positions = []
        for _ in range(attempts):
            with self.image_lock:
                frame = self.cv_image.copy()
                
            detected, pos, area = self.detect_ffb(frame)
            if detected:
                ffb_positions.append(pos[1])  # Y-coordinate only
            
            time.sleep(0.1)  # Short delay between attempts
        
        if not ffb_positions:
            self.get_logger().warn('No FFB detected for centering')
            return False
        
        # Calculate average FFB position
        avg_ffb_y = sum(ffb_positions) / len(ffb_positions)
        y_error = avg_ffb_y - frame_center_y
        
        if abs(y_error) <= threshold:
            self.get_logger().info('FFB already centered')
        else:
        
            # Calculate servo angle adjustment
            # Convert pixel error to angle (adjust multiplier based on your setup)
            angle_adjustment = -y_error * 0.1  # Negative because camera movement is inverse

            angle_adjustment = np.clip(angle_adjustment, 0, np.pi)
            
            # Send servo command
            msg = Float64MultiArray()
            msg.data = [float(angle_adjustment)]
            self.servo_publisher.publish(msg)
            
            self.get_logger().info(f'Adjusted servo by {angle_adjustment:.2f} degrees')
            
            # # Verify centering
            # ret, frame = self.camera.read()
            # if ret:
            #     detected, pos, area = self.detect_ffb(frame)
            #     if detected:
            #         final_error = abs(pos[1] - frame_center_y)
            #         if final_error <= threshold:
            #             self.get_logger().info('FFB centered successfully')
            #             return True
                    
            # self.get_logger().warn('Failed to center FFB')
            

    def start_recording(self, tree_id):
        if self.recording:
            return False

        self.current_tree_id = tree_id
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        video_path = os.path.join(
            'results',
            f'tree_{tree_id}_{timestamp}.mp4'
        )

        try:
            self.video_writer = cv2.VideoWriter(
                video_path,
                cv2.VideoWriter_fourcc(*'XVID'),
                30.0,
                (1920, 1080)
            )
            self.recording = True
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {e}')
            return False

    def stop_recording(self):
        """Stop recording"""
        if not self.recording:
            return
            
        self.recording = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            
        # Return servo to center position
        msg = Float64MultiArray()
        msg.data = [0.0]
        self.servo_publisher.publish(msg)
        
        self.get_logger().info('Stopped recording and reset servo')


def main(args=None):
    rclpy.init(args=args)
    camera_control_node = CameraController()
    rclpy.spin(camera_control_node)
    camera_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()