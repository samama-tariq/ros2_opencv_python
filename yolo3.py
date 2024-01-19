import cv2
import torch
import os

# Constants
CONFIDENCE_THRESHOLD = 0.4
RECTANGLE_SIZE = 25
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1
FONT_COLOR = (255, 0, 0)
FONT_THICKNESS = 1

# Global variables
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print("Using Device: ", device)
tracker = None
tracking = False
current_frame = None
bbox = None

# Function to load YOLOv5 model
def load_model(model_name):
    local_yolov5_path = 'D:/woot_tech_working/YOLOV5_WORKING_ROS2/yolov5'
    if not os.path.isfile(model_name):
        raise ValueError(f"Model file not found: {model_name}")
    model = torch.hub.load(local_yolov5_path, 'custom', path=model_name, source='local')
    return model

# Function to perform YOLOv5 inference
def score_frame(model, frame):
    model.to(device)
    frame = [frame]
    results = model(frame)
    labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
    detections = []
    for label, bbox in zip(labels, cord):
        detection = {'id': int(label), 'bbox': bbox.tolist()}
        detections.append(detection)
    return detections

# Function to draw rectangles on the frame
def plot_boxes(detections, frame):
    if not detections:
        return frame
    for detection in detections:
        bbox = detection['bbox']
        confidence = bbox[4]
        if confidence >= CONFIDENCE_THRESHOLD:
            x_shape, y_shape = frame.shape[1], frame.shape[0]
            x1, y1, x2, y2 = int(bbox[0]*x_shape), int(bbox[1]*y_shape), int(bbox[2]*x_shape), int(bbox[3]*y_shape)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return frame

# Function to initialize and update CSRT tracker
def update_tracker(tracker, frame):
    success, bbox = tracker.update(frame)
    return success, bbox

# Function to handle mouse events
def mouse_callback(event, x, y, flags, param):
    global tracker, tracking, bbox, current_frame
    if event == cv2.EVENT_LBUTTONDOWN and not tracking:
        bbox = (x - 12, y - 12, RECTANGLE_SIZE, RECTANGLE_SIZE)  # x, y, width, height
        tracker = cv2.TrackerCSRT_create()
        tracker.init(current_frame, bbox)
        tracking = True

# Main program
if __name__ == "__main__":
    # capture_index = 'wakeb.mp4'
    capture_index = 0
    model_name = 'visdrone.pt'
    model = load_model(model_name)

    cap = cv2.VideoCapture(capture_index)
    if not cap.isOpened():
        print("Error opening video stream or file")
        exit(1)

    cv2.namedWindow('YOLOv5 Detection')
    cv2.setMouseCallback('YOLOv5 Detection', mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame, ending detection")
            break

        current_frame = cv2.resize(frame, (640, 480))
        detections = score_frame(model, current_frame)
        current_frame = plot_boxes(detections, current_frame)

        if tracking:
            success, bbox = update_tracker(tracker, current_frame)
            tracking_label = cv2.putText(current_frame, 'TRACKING', (50, 80), FONT, FONT_SCALE, FONT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
            if success:
                x, y, w, h = map(int, bbox)
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.circle(current_frame, (center_x, center_y), 2, (0, 0, 255), -1)

        raw_label = cv2.putText(current_frame, 'RAW VIDEO', (50, 50), FONT, FONT_SCALE, FONT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
        cv2.imshow('YOLOv5 Detection', current_frame)

        key = cv2.waitKey(5) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):  # Reset tracker when 'r' is pressed
            tracking = False
            tracker = None

    cap.release()
    cv2.destroyAllWindows()
