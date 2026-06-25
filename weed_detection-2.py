import cv2
from ultralytics import YOLO
import os
import csv
import time

# =========================
# SETTINGS
# =========================
MODEL_PATH = "/home/lior/best.pt"
VIDEO_SOURCE = 0
CONFIDENCE_THRESHOLD = 0.7

FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720

# 0=watermelon, 1=weed_side, 2=weed_top
DATASET_NAMES = {0: "watermelon", 1: "weed_side", 2: "weed_top"}

SOIL_OFFSET_FACTOR = 0.08
TOP_Y_FACTOR = 0.55

FIRE_ZONE_W_FACTOR = 0.05
FIRE_ZONE_H_FACTOR = 0.05

LOG_FILE = "weed_target.csv"


def run_realtime_detection(no_display: bool = False):
    if not os.path.exists(MODEL_PATH):
        print(f"Model file not found: {MODEL_PATH}")
        return

    print(f"Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print(f"Could not open video source {VIDEO_SOURCE}")
        return

    flip_1_2 = False

    def remap_id(raw_id: int) -> int:
        if not flip_1_2:
            return raw_id
        if raw_id == 1:
            return 2
        if raw_id == 2:
            return 1
        return raw_id

    with open(LOG_FILE, mode="w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow([
            "timestamp",
            "raw_id", "mapped_id", "mapped_name",
            "confidence",
            "cx", "cy",
            "fire_allowed",
            "x1", "y1", "x2", "y2",
            "flip_mode"
        ])

        print("Running. Press 'f' to toggle flip 1<->2, press 'q' to quit.")

        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                break

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            result = model.predict(source=frame, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]
            annotated = result.plot()

            h, w, _ = annotated.shape
            center_x, center_y = w // 2, h // 2
            fire_zone_w = int(w * FIRE_ZONE_W_FACTOR)
            fire_zone_h = int(h * FIRE_ZONE_H_FACTOR)

            best_box = None
            best_conf = 0.0
            best_raw_id = None
            best_mapped_id = None

            if result.boxes is not None and len(result.boxes) > 0:
                for b in result.boxes:
                    raw_id = int(b.cls[0])
                    mapped_id = remap_id(raw_id)
                    conf = float(b.conf[0])
                    if mapped_id not in (1, 2):
                        continue
                    if conf > best_conf:
                        best_conf = conf
                        best_box = b
                        best_raw_id = raw_id
                        best_mapped_id = mapped_id

            if best_box is not None:
                x1, y1, x2, y2 = map(int, best_box.xyxy[0])
                box_h = y2 - y1
                cx = int((x1 + x2) / 2)
                mapped_name = DATASET_NAMES.get(best_mapped_id, "unknown")

                if best_mapped_id == 1:
                    cy = int(y2 - SOIL_OFFSET_FACTOR * box_h)
                else:
                    cy = int(y1 + TOP_Y_FACTOR * box_h)

                cy = max(0, min(cy, h - 1))
                cv2.circle(annotated, (cx, cy), 7, (0, 0, 255), -1)

                dx = abs(cx - center_x)
                dy = abs(cy - center_y)
                fire_allowed = (dx < fire_zone_w and dy < fire_zone_h)
                status = "FIRE" if fire_allowed else "NO FIRE"
                color = (0, 0, 255) if fire_allowed else (0, 255, 0)

                cv2.putText(
                    annotated,
                    f"{status} ({mapped_name}) {best_conf:.2f}",
                    (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3
                )

                writer.writerow([
                    time.time(),
                    best_raw_id, best_mapped_id, mapped_name,
                    best_conf, cx, cy,
                    int(fire_allowed),
                    x1, y1, x2, y2,
                    int(flip_1_2)
                ])

            else:
                # Show fire-zone box only when scanning (no detection)
                cv2.rectangle(
                    annotated,
                    (center_x - fire_zone_w, center_y - fire_zone_h),
                    (center_x + fire_zone_w, center_y + fire_zone_h),
                    (255, 0, 0), 2
                )
                cv2.putText(
                    annotated, "NO WEED", (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3
                )

            if not no_display:
                cv2.imshow("YOLO Weed Detection", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("f"):
                flip_1_2 = not flip_1_2

    cap.release()
    cv2.destroyAllWindows()
    print("Finished. CSV saved to:", LOG_FILE)


def run_ros_mode(topic: str, model_path: str, confidence: float, no_display: bool = False):
    """Subscribe to a ROS 2 image topic and run detection, publishing results back."""
    import queue
    import threading

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage
    from vision_msgs.msg import (Detection2D, Detection2DArray,
                                 ObjectHypothesisWithPose)

    if not os.path.exists(model_path):
        print(f"Model file not found: {model_path}")
        return

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)
    bridge = CvBridge()
    frame_queue: queue.Queue = queue.Queue(maxsize=2)

    flip_1_2 = False

    def remap_id(raw_id: int) -> int:
        if not flip_1_2:
            return raw_id
        if raw_id == 1:
            return 2
        if raw_id == 2:
            return 1
        return raw_id

    class WeedDetectionNode(Node):
        def __init__(self):
            super().__init__('weed_detection')
            self._sub = self.create_subscription(
                RosImage, topic, self._image_cb, 10)
            self.det_pub = self.create_publisher(
                Detection2DArray, '/detections', 10)
            self.img_pub = self.create_publisher(
                RosImage, '/detection_image', 10)
            self.get_logger().info(
                f'weed_detection ready — subscribed to {topic}')

        def _image_cb(self, msg: RosImage):
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Drop oldest frame if consumer hasn't caught up
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put_nowait(frame)

    rclpy.init()
    node = WeedDetectionNode()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"ROS mode — topic: {topic}. Press 'f' to flip 1<->2, 'q' to quit.")

    try:
        while rclpy.ok():
            try:
                frame = frame_queue.get(timeout=0.1)
            except queue.Empty:
                if not no_display and cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            result = model.predict(source=frame, conf=confidence, verbose=False)[0]
            annotated = result.plot()

            h, w, _ = annotated.shape
            center_x, center_y = w // 2, h // 2
            fire_zone_w = int(w * FIRE_ZONE_W_FACTOR)
            fire_zone_h = int(h * FIRE_ZONE_H_FACTOR)

            det_array = Detection2DArray()
            best_box = None
            best_conf = 0.0
            best_raw_id = None
            best_mapped_id = None

            if result.boxes is not None and len(result.boxes) > 0:
                for b in result.boxes:
                    raw_id = int(b.cls[0])
                    mapped_id = remap_id(raw_id)
                    conf = float(b.conf[0])
                    if mapped_id not in (1, 2):
                        continue
                    if conf > best_conf:
                        best_conf = conf
                        best_box = b
                        best_raw_id = raw_id
                        best_mapped_id = mapped_id

            if best_box is not None:
                x1, y1, x2, y2 = map(int, best_box.xyxy[0])
                box_h = y2 - y1
                cx = int((x1 + x2) / 2)
                mapped_name = DATASET_NAMES.get(best_mapped_id, "unknown")

                if best_mapped_id == 1:
                    cy = int(y2 - SOIL_OFFSET_FACTOR * box_h)
                else:
                    cy = int(y1 + TOP_Y_FACTOR * box_h)
                cy = max(0, min(cy, h - 1))

                cv2.circle(annotated, (cx, cy), 7, (0, 0, 255), -1)

                dx = abs(cx - center_x)
                dy = abs(cy - center_y)
                fire_allowed = (dx < fire_zone_w and dy < fire_zone_h)
                status = "FIRE" if fire_allowed else "NO FIRE"
                color = (0, 0, 255) if fire_allowed else (0, 255, 0)

                cv2.putText(
                    annotated,
                    f"{status} ({mapped_name}) {best_conf:.2f}",
                    (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3
                )

                det = Detection2D()
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(box_h)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(best_mapped_id)
                hyp.hypothesis.score = best_conf
                det.results.append(hyp)
                det_array.detections.append(det)

            else:
                # Show fire-zone box only when scanning (no detection)
                cv2.rectangle(
                    annotated,
                    (center_x - fire_zone_w, center_y - fire_zone_h),
                    (center_x + fire_zone_w, center_y + fire_zone_h),
                    (255, 0, 0), 2
                )
                cv2.putText(
                    annotated, "NO WEED", (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3
                )

            node.det_pub.publish(det_array)
            node.img_pub.publish(
                bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

            if not no_display:
                cv2.imshow("YOLO Weed Detection", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('f'):
                    flip_1_2 = not flip_1_2

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    print("ROS mode finished.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Weed detection with YOLO")
    parser.add_argument(
        '--topic', type=str, default=None,
        help='ROS 2 image topic to subscribe to (enables ROS mode). '
             'E.g. /camera/image_raw (sim) or /camera/color/image_raw (real)')
    parser.add_argument(
        '--model', type=str, default=None,
        help='Path to YOLO .pt weights file (overrides MODEL_PATH)')
    parser.add_argument(
        '--confidence', type=float, default=CONFIDENCE_THRESHOLD,
        help='Detection confidence threshold (default: %(default)s)')
    parser.add_argument(
        '--no-display', action='store_true',
        help='Suppress the OpenCV preview window (useful when running headless)')
    args = parser.parse_args()

    if args.model:
        MODEL_PATH = args.model

    if args.topic:
        run_ros_mode(args.topic, MODEL_PATH, args.confidence,
                     no_display=args.no_display)
    else:
        run_realtime_detection(no_display=args.no_display)
