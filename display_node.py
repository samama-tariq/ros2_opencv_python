# Adjusted Display Node Script
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture_node')
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.cap = cv2.VideoCapture('rtsp://127.0.0.1:8554/test')
        self.publisher_ = self.create_publisher(String, 'rectangle_coordinates', 10)
        self.drawing = False
        self.rect_start = None
        self.rect_end = None

        cv2.namedWindow('Video Capture')
        cv2.setMouseCallback('Video Capture', self.mouse_events)

    def mouse_events(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.rect_start = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.rect_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.rect_end = (x, y)
            coords = json.dumps({'start': self.rect_start, 'end': self.rect_end})
            self.publisher_.publish(String(data=coords))
            self.get_logger().info('Publishing coordinates: {}'.format(coords))

    def draw_rectangle(self, frame):
        if self.rect_start and self.rect_end:
            cv2.rectangle(frame, self.rect_start, self.rect_end, (0, 255, 0), 2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (640, 480))
            self.draw_rectangle(frame)
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
