import cv2
import time
import mediapipe as mp
from mediapipe.tasks.python import vision

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.i += 1

rclpy.init(args=args)

minimal_publisher = MinimalPublisher()

rclpy.spin(minimal_publisher)

# Destroy the node explicitly
# (optional - otherwise it will be done automatically
# when the garbage collector destroys the node object)
minimal_publisher.destroy_node()
rclpy.shutdown()

# -----------------------------
# MediaPipe setupq
# -----------------------------
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="hand_wave_detection-1/hand_landmarker.task"),
    running_mode=VisionRunningMode.IMAGE
)

landmarker = HandLandmarker.create_from_options(options)

# -----------------------------
# Camera
# -----------------------------
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 10)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# -----------------------------
# Wave zones and parameters
# -----------------------------
zones = {
    "whole_frame": (0.0, 1.0)
}
# zones = {
#     "left": (0.05, 0.35),
#     "right": (0.65, 0.95)
# }

wiggle_distance = 0.015
crosses_min = 1
smooth_alpha = 0.3
display_time = 1.5

# Static (held hand) parameters
static_time_held = 2.0
static_move_threshold = 0.005

# -----------------------------
# HandWaveDetector class
# -----------------------------
class HandWaveDetector:
    def __init__(self):
        self.counters = {"whole_frame": 0}
        self.last_x = {"whole_frame": None}
        self.direction = {"whole_frame": None}
        # self.counters = {"left": 0, "right": 0}
        # self.last_x = {"left": None, "right": None}
        # self.direction = {"left": None, "right": None}

    def update(self, x, zone_name):
        if self.last_x[zone_name] is None:
            self.last_x[zone_name] = x
            return False

        waving_detected = False
        delta = x - self.last_x[zone_name]

        if abs(delta) > wiggle_distance:
            new_dir = "right" if delta > 0 else "left"
            if self.direction[zone_name] and new_dir != self.direction[zone_name]:
                self.counters[zone_name] += 1
                if self.counters[zone_name] >= crosses_min:
                    waving_detected = True
                    self.counters[zone_name] = 0
                    self.direction[zone_name] = None
            else:
                self.direction[zone_name] = new_dir

        self.last_x[zone_name] = x
        return waving_detected

# -----------------------------
# State variables
# -----------------------------
wave_detector = HandWaveDetector()
smoothed_x = None
# last_wave_time = {"left": 0, "right": 0}
# static_start_time = {"left": None, "right": None}

last_wave_time = {"whole_frame": 0}
static_start_time = {"whole_frame": None}

# -----------------------------
# Main loop
# -----------------------------
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    mp_image = mp.Image(
        image_format=mp.ImageFormat.SRGB,
        data=rgb
    )

    result = landmarker.detect(mp_image)

    # Check each zone
    waving_detected = False

    for name, (x_min, x_max) in zones.items():
        # if x_min <= smoothed_x <= x_max:

        # Wave detection
        # if wave_detector.update(smoothed_x, name):
        #     last_wave_time[name] = time.time()
        #     static_start_time[name] = None
        #     waving_detected = True

        zone_x1 = int(x_min * frame_width)
        zone_x2 = int(x_max * frame_width)
        cv2.rectangle(frame, (zone_x1, 0), (zone_x2, frame_height), (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"{name.upper()} ZONE",
            (zone_x1 + 10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

    waving_detected = False

    if result.hand_landmarks:
        landmarks = result.hand_landmarks[0]

        # Average x of palm landmarks
        palm_ids = [0, 1, 5, 9, 13, 17]
        x = sum(landmarks[i].x for i in palm_ids) / len(palm_ids)

        # Smooth x
        if smoothed_x is None:
            smoothed_x = x
        else:
            smoothed_x = smooth_alpha * x + (1 - smooth_alpha) * smoothed_x

        # Check each zone
        for name, (x_min, x_max) in zones.items():
            if x_min <= smoothed_x <= x_max:

                # Original wave detection
                if wave_detector.update(smoothed_x, name):
                    last_wave_time[name] = time.time()
                    static_start_time[name] = None
                    waving_detected = True

                # Static (held hand) detection
                else:
                    last_x = wave_detector.last_x[name]

                    if last_x is not None and abs(smoothed_x - last_x) < static_move_threshold:
                        if static_start_time[name] is None:
                            static_start_time[name] = time.time()
                        elif time.time() - static_start_time[name] >= static_time_held:
                            last_wave_time[name] = time.time()
                            static_start_time[name] = None
                            waving_detected = True
                    else:
                        static_start_time[name] = None
            else:
                static_start_time[name] = None

        # Draw palm landmarks
        for i in palm_ids:
            px = int(landmarks[i].x * frame_width)
            py = int(landmarks[i].y * frame_height)
            cv2.circle(frame, (px, py), 8, (0, 255, 255), -1)
        
        # # print x,y coordinates
        # for i in palm_ids:
        #     x = landmarks[i].x
        #     y = landmarks[i].y
        #     z = landmarks[i].z

        #     print(f"Landmark {i}: x={x:.4f}, y={y:.4f}, z={z:.4f}")
        
        # 1. Calculate the average (centroid) x, y, and z coordinates of the palm
        palm_center_x = sum(landmarks[i].x for i in palm_ids) / len(palm_ids)
        palm_center_y = sum(landmarks[i].y for i in palm_ids) / len(palm_ids)
        palm_center_z = sum(landmarks[i].z for i in palm_ids) / len(palm_ids)

        # 2. Print the normalized 3D coordinates of the palm center
        if waving_detected:
            print(
                f"Palm Center -> "
                f"x: {palm_center_x:.4f}, "
                f"y: {palm_center_y:.4f}, "
                f"z: {palm_center_z:.4f}"
            )

    # Show WAVING text
    for idx, name in enumerate(zones.keys()):
        if time.time() - last_wave_time[name] < display_time:
            cv2.putText(
                frame,
                "WAVING",
                (50, 100 + 50 * idx),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (0, 255, 0),
                3
            )

    cv2.imshow("MediaPipe Palm Wave Detector", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# -----------------------------
# Cleanup
# -----------------------------
cap.release()
cv2.destroyAllWindows()
landmarker.close()
