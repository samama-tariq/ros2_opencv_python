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
        self.guidance_publisher = self.create_publisher(String, 'guidance_command', 10)
        self.rect_center = None
        self.rect_size = 25  # Size of the square rectangle

        cv2.namedWindow('Video Capture')
        cv2.setMouseCallback('Video Capture', self.mouse_events)

    def mouse_events(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.rect_center = (x, y)
            self.publish_rectangle_coordinates()

    def publish_rectangle_coordinates(self):
        if self.rect_center:
            x, y = self.rect_center
            half_size = self.rect_size // 2
            coords = json.dumps({
                'start': (x - half_size, y - half_size),
                'end': (x + half_size, y + half_size)
            })
            self.publisher_.publish(String(data=coords))
            self.get_logger().info('Publishing coordinates: {}'.format(coords))

    def draw_rectangle(self, frame):
        if self.rect_center:
            x, y = self.rect_center
            half_size = self.rect_size // 2
            cv2.rectangle(frame, (x - half_size, y - half_size), (x + half_size, y + half_size), (0, 255, 0), 2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (640, 480))
            self.draw_rectangle(frame)
            cv2.imshow('Video Capture', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.destroy_node()
                cv2.destroyAllWindows()
            elif key == ord('x'):
                self.guidance_publisher.publish(String(data='guidance on'))
                print('x press')

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
