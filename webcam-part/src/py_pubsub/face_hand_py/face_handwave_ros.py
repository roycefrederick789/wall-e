import os
import time
import urllib.request
import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node

# Import custom messages from your interfaces package
from tutorial_interfaces.msg import FaceDetection, HandDetection


class HandWaveDetector:
    def __init__(self, wiggle_distance=0.015, crosses_min=1):
        self.wiggle_distance = wiggle_distance
        self.crosses_min = crosses_min
        self.counters = {"whole_frame": 0}
        self.last_x = {"whole_frame": None}
        self.direction = {"whole_frame": None}

    def update(self, x, zone_name="whole_frame"):
        if self.last_x[zone_name] is None:
            self.last_x[zone_name] = x
            return False

        waving_detected = False
        delta = x - self.last_x[zone_name]

        if abs(delta) > self.wiggle_distance:
            new_dir = "right" if delta > 0 else "left"
            if self.direction[zone_name] and new_dir != self.direction[zone_name]:
                self.counters[zone_name] += 1
                if self.counters[zone_name] >= self.crosses_min:
                    waving_detected = True
                    self.counters[zone_name] = 0
                    self.direction[zone_name] = None
            else:
                self.direction[zone_name] = new_dir

        self.last_x[zone_name] = x
        return waving_detected


class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        # ----------------------------------------------------
        # ROS 2 Publishers
        # ----------------------------------------------------
        self.face_pub = self.create_publisher(FaceDetection, 'face_coord_host2laptop', 10)
        self.hand_pub = self.create_publisher(HandDetection, 'hand_coord_host2laptop', 10)

        # ----------------------------------------------------
        # MediaPipe Model Download & Setup
        # ----------------------------------------------------
        model_dir = "face_detection"
        os.makedirs(model_dir, exist_ok=True)
        self.face_model_path = os.path.join(model_dir, "blaze_face_short_range.tflite")
        if not os.path.exists(self.face_model_path):
            self.get_logger().info("Downloading BlazeFace Short Range model...")
            model_url = (
                "https://storage.googleapis.com/mediapipe-models/"
                "face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite"
            )
            urllib.request.urlretrieve(model_url, self.face_model_path)

        BaseOptions = mp.tasks.BaseOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        # Face Detector
        face_options = mp.tasks.vision.FaceDetectorOptions(
            base_options=BaseOptions(model_asset_path=self.face_model_path),
            running_mode=VisionRunningMode.IMAGE,
            min_detection_confidence=0.5
        )
        self.face_detector = mp.tasks.vision.FaceDetector.create_from_options(face_options)

        # Hand Landmarker
        hand_model_path = "hand_wave_detection-1/hand_landmarker.task"
        hand_options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=hand_model_path),
            running_mode=VisionRunningMode.IMAGE
        )
        self.landmarker = mp.tasks.vision.HandLandmarker.create_from_options(hand_options)

        # ----------------------------------------------------
        # Camera Setup
        # ----------------------------------------------------
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Tracking state
        self.zones = {"whole_frame": (0.0, 1.0)}
        self.smooth_alpha = 0.3
        self.smoothed_x = None
        self.wave_detector = HandWaveDetector(wiggle_distance=0.015, crosses_min=1)
        self.last_wave_time = {"whole_frame": 0}
        self.static_start_time = {"whole_frame": None}
        self.static_time_held = 2.0
        self.static_move_threshold = 0.005
        self.palm_ids = [0, 1, 5, 9, 13, 17]

        # ----------------------------------------------------
        # Loop Timer (~30 FPS)
        # ----------------------------------------------------
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info("Vision Detection Node initialized and running.")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture image frame from camera.")
            return

        frame = cv2.flip(frame, 1)
        frame_h, frame_w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # ----------------------------------------------------
        # 1. Process Face Detection & Publish
        # ----------------------------------------------------
        face_msg = FaceDetection()
        face_result = self.face_detector.detect(mp_image)

        if face_result.detections:
            face_msg.detected = True
            primary_face = face_result.detections[0].bounding_box
            center_x = primary_face.origin_x + primary_face.width / 2.0
            center_y = primary_face.origin_y + primary_face.height / 2.0
            face_msg.x = float(center_x / frame_w)
            face_msg.y = float(center_y / frame_h)

            # UI drawing
            cv2.rectangle(
                frame,
                (primary_face.origin_x, primary_face.origin_y),
                (primary_face.origin_x + primary_face.width, primary_face.origin_y + primary_face.height),
                (255, 0, 0), 2
            )
            cv2.putText(frame, "FACE DETECTED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        else:
            face_msg.detected = False
            face_msg.x = 0.0
            face_msg.y = 0.0

        self.face_pub.publish(face_msg)

        # ----------------------------------------------------
        # 2. Process Hand Detection & Publish
        # ----------------------------------------------------
        hand_msg = HandDetection()
        hand_result = self.landmarker.detect(mp_image)
        waving_detected = False

        if hand_result.hand_landmarks:
            landmarks = hand_result.hand_landmarks[0]
            palm_x = sum(landmarks[i].x for i in self.palm_ids) / len(self.palm_ids)
            palm_y = sum(landmarks[i].y for i in self.palm_ids) / len(self.palm_ids)

            # Smooth X
            if self.smoothed_x is None:
                self.smoothed_x = palm_x
            else:
                self.smoothed_x = self.smooth_alpha * palm_x + (1 - self.smooth_alpha) * self.smoothed_x

            # Wave check
            for name, (x_min, x_max) in self.zones.items():
                if x_min <= self.smoothed_x <= x_max:
                    if self.wave_detector.update(self.smoothed_x, name):
                        self.last_wave_time[name] = time.time()
                        self.static_start_time[name] = None
                        waving_detected = True
                    else:
                        last_x = self.wave_detector.last_x[name]
                        if last_x is not None and abs(self.smoothed_x - last_x) < self.static_move_threshold:
                            if self.static_start_time[name] is None:
                                self.static_start_time[name] = time.time()
                            elif time.time() - self.static_start_time[name] >= self.static_time_held:
                                self.last_wave_time[name] = time.time()
                                self.static_start_time[name] = None
                                waving_detected = True
                        else:
                            self.static_start_time[name] = None

            hand_msg.x = float(palm_x)
            hand_msg.y = float(palm_y)
            hand_msg.waving = waving_detected

            # Draw palm landmarks
            for idx in self.palm_ids:
                px = int(landmarks[idx].x * frame_w)
                py = int(landmarks[idx].y * frame_h)
                cv2.circle(frame, (px, py), 6, (0, 255, 255), -1)
        else:
            self.smoothed_x = None
            hand_msg.x = 0.0
            hand_msg.y = 0.0
            hand_msg.waving = False

        self.hand_pub.publish(hand_msg)

        # ----------------------------------------------------
        # 3. OpenCV Display & UI
        # ----------------------------------------------------
        if time.time() - self.last_wave_time["whole_frame"] < 1.5:
            cv2.putText(frame, "WAVING", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

        cv2.imshow("MediaPipe Face + Hand Node", frame)
        
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.get_logger().info("Shutdown signal received from GUI.")
        #     rclpy.shutdown()
        #     break
        # Replace your old waitKey check with this:
        key = cv2.waitKey(1) & 0xFF

        # 1. Close if 'q' is pressed
        # 2. Close if 'ESC' (ASCII 27) is pressed
        # 3. Close if user clicks the window's top-right red "X" button
        if (key == ord('q') or
            key == 27 or
            cv2.getWindowProperty("MediaPipe Face + Hand Node", cv2.WND_PROP_VISIBLE) < 1):

            self.get_logger().info("Shutdown signal received. Closing...")
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()

        # Add these two lines to flush the GUI thread and force close the window
        cv2.waitKey(1)
        cv2.waitKey(1)

        self.landmarker.close()
        self.face_detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
