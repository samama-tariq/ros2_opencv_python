import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Point
import json
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'rectangle_coordinates',
            self.listener_callback,
            10)
        self.guidance_subscriber = self.create_subscription(
            String,
            'guidance_command',
            self.guidance_callback,
            10)
        self.change_mode_flag = False

        self.rectangle_pub = self.create_publisher(Int32MultiArray, 'rectangle_data', 10)
        self.circle_pub = self.create_publisher(Point, 'circle_data', 10)
        self.text_pub = self.create_publisher(String, 'text_data', 10)

        self.cap = cv2.VideoCapture('rtsp://127.0.0.1:8554/test')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # CSRT Tracker
        self.tracker = None
        self.tracking = False

        # MAVLink Connection
        self.mode = 'None'
        self.master = mavutil.mavlink_connection('tcp:192.168.0.106:14550')
        print("Connected to MAVLink!") 
        print("System ID:", self.master.target_system)
        print("Component ID:", self.master.target_component)
        self.boot_time = time.time()

        # Start the video capture in the ROS2 loop
        self.timer = self.create_timer(0.01, self.capture_video)

    def guidance_callback(self, msg):
        if msg.data == 'guidance on':
            self.change_mode_flag = True
            print("Received 'guidance on' command, changing mode to GUIDED.")

    def x_map(self, x, in_min, in_max, out_min, out_max):
        if x > -0.22 and x < 0.22:
            return int(0)
        else:
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def y_map(self, x, in_min, in_max, out_min, out_max):
        if x > -0.2 and x < 0.2:
            msg = self.master.recv_msg()
            if msg is not None:
                pitch_ = math.degrees(msg.pitch)
                return pitch_
            else:
                return int(-22)
        else:
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def capture_video(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return  # Skip empty frames

        frame = cv2.resize(frame, (640, 480))
        (H, W) = frame.shape[:2]
        frame_x = (W / 2)
        frame_y = (H / 2)

        try:
            if self.tracking and self.tracker is not None:
                success, box = self.tracker.update(frame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    rect_msg = Int32MultiArray()
                    rect_msg.data = [x, y, w, h]
                    self.rectangle_pub.publish(rect_msg)

                    center_x = x + w // 2
                    center_y = y + h // 2
                    cv2.circle(frame, (center_x, center_y), 2, (0, 0, 255), -1)
                    circle_msg = Point()
                    circle_msg.x = float(center_x)
                    circle_msg.y = float(center_y)
                    circle_msg.z = 0.0  # z can be ignored or used for radius
                    self.circle_pub.publish(circle_msg)

                    bbox_x = int(x + w // 2)
                    bbox_y = int(y + h // 2)

                    x_p = bbox_x - frame_x
                    y_p = bbox_y - frame_y
                    roll = 0.017 * x_p
                    pitch = 0.02 * y_p
                    x_val = self.x_map(roll, -10, 10, -10, 10)
                    y_val = self.y_map(pitch, -7, 7, -12, -32)
                    print('node start tracking')

                    if self.mode == 'GUIDED':
                        print('OBC Taking Over')
                        font = cv2.FONT_HERSHEY_SIMPLEX 
                        org = (50, 110) 
                        fontScale = 1
                        color = (255, 0, 0)
                        thickness = 1
                        frame = cv2.putText(frame, 'GUIDED', org, font, fontScale, color, thickness, cv2.LINE_AA)
                        text_msg = String()
                        text_msg.data = 'GUIDED'
                        self.text_pub.publish(text_msg)
                        self.master.mav.set_attitude_target_send(
                            int(1e3*(time.time()-self.boot_time)),
                            self.master.target_system,
                            self.master.target_component,
                            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
                            QuaternionBase([math.radians(angle) for angle in (x_val, y_val, 0)]),
                            0, 0, 0, 0)
                else:
                    self.get_logger().error('Tracker update failed')
        except Exception as e:
            self.get_logger().error('Error in tracking: {}'.format(str(e)))

        #cv2.imshow('Tracker Node', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            self.cap.release()
            cv2.destroyAllWindows()
        
        if self.change_mode_flag:
                self.mode = 'GUIDED'
                mode_id = self.master.mode_mapping()[self.mode]
                self.master.set_mode(mode_id)
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)
                self.change_mode_flag = False

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "{}"'.format(msg.data))
        try:
            data = json.loads(msg.data)
            start_point = tuple(data['start'])
            end_point = tuple(data['end'])
            bbox = (start_point[0], start_point[1], end_point[0] - start_point[0], end_point[1] - start_point[1])

            # Check if a tracker is already initialized
            if self.tracker is not None:
                self.tracking = False
                self.tracker = None  # Release the current tracker

            # Read a frame from the capture
            ret, frame = self.cap.read()
            if ret and frame is not None:
                # Initialize the CSRT Tracker with the new rectangle
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(frame, bbox)
                self.tracking = True
            else:
                self.get_logger().error('Failed to initialize tracker due to empty frame')
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse JSON string')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
