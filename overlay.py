import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Point
import json

class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture_node')
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.cap = cv2.VideoCapture('rtsp://127.0.0.1:8554/test')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        #self.publisher_ = self.create_publisher(String, 'rectangle_coordinates', 10)
        #self.drawing = False
        #self.rect_start = None
        #self.rect_end = None

        # Subscriptions for overlay data
        self.rectangle_sub = self.create_subscription(Int32MultiArray, 'rectangle_data', self.rectangle_callback, 10)
        self.circle_sub = self.create_subscription(Point, 'circle_data', self.circle_callback, 10)
        self.text_sub = self.create_subscription(String, 'text_data', self.text_callback, 10)

        # Variables to store received data
        self.received_rectangle = None
        self.received_circle = None
        self.received_text = None

        cv2.namedWindow('Video Capture')
        # cv2.setMouseCallback('Video Capture', self.mouse_events)

    def rectangle_callback(self, msg):
        self.received_rectangle = msg.data  # [x, y, w, h]

    def circle_callback(self, msg):
        self.received_circle = (int(msg.x), int(msg.y))

    def text_callback(self, msg):
        self.received_text = msg.data

    def draw_received_shapes(self, frame):
        # Draw received rectangle
        if self.received_rectangle:
            x, y, w, h = self.received_rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Draw received circle
        if self.received_circle:
            cv2.circle(frame, self.received_circle, 2, (0, 0, 255), -1)

        # Draw received text
        if self.received_text:
            cv2.putText(frame, self.received_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # def mouse_events(self, event, x, y, flags, param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         self.drawing = True
    #         self.rect_start = (x, y)
    #     elif event == cv2.EVENT_MOUSEMOVE:
    #         if self.drawing:
    #             self.rect_end = (x, y)
    #     elif event == cv2.EVENT_LBUTTONUP:
    #         self.drawing = False
    #         self.rect_end = (x, y)
    #         # Publish the coordinates
    #         coords = json.dumps({'start': self.rect_start, 'end': self.rect_end})
    #         self.publisher_.publish(String(data=coords))

    # def draw_rectangle(self, frame):
    #     if self.rect_start and self.rect_end:
    #         cv2.rectangle(frame, self.rect_start, self.rect_end, (0, 255, 0), 2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (640, 480))
            #self.draw_rectangle(frame)
            self.draw_received_shapes(frame)  # Draw shapes received from tracker
            cv2.imshow('Video Capture', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    video_capture_node = VideoCaptureNode()
    try:
        rclpy.spin(video_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
