import cv2
from ultralytics import YOLO
import os
import csv
import time

# =========================
# SETTINGS
# =========================
MODEL_PATH = r"C:\Users\liore\Weed_control\version3\best.pt"
VIDEO_SOURCE = 1
CONFIDENCE_THRESHOLD = 0.7

FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720

# Your dataset meaning:
# 0=watermelon, 1=weed_side, 2=weed_top
DATASET_NAMES = {0: "watermelon", 1: "weed_side", 2: "weed_top"}

# Target point logic
SOIL_OFFSET_FACTOR = 0.08
TOP_Y_FACTOR = 0.55

# Fire zone size
FIRE_ZONE_W_FACTOR = 0.05
FIRE_ZONE_H_FACTOR = 0.05

LOG_FILE = "weed_target.csv"


def run_realtime_detection():
    if not os.path.exists(MODEL_PATH):
        print(f"Model file not found: {MODEL_PATH}")
        return

    print(f"Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print(f"Could not open video source {VIDEO_SOURCE}")
        return

    # Press 'f' to swap class 1<->2 during runtime
    flip_1_2 = False

    def remap_id(raw_id: int) -> int:
        """Optional swap between class 1 and 2."""
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
            annotated = result.plot()  # draws normal YOLO boxes + labels

            h, w, _ = annotated.shape
            center_x, center_y = w // 2, h // 2
            fire_zone_w = int(w * FIRE_ZONE_W_FACTOR)
            fire_zone_h = int(h * FIRE_ZONE_H_FACTOR)

            # Fire zone rectangle
            cv2.rectangle(
                annotated,
                (center_x - fire_zone_w, center_y - fire_zone_h),
                (center_x + fire_zone_w, center_y + fire_zone_h),
                (255, 0, 0),
                2
            )

            # Choose best weed by confidence (using mapped IDs)
            best_box = None
            best_conf = 0.0
            best_raw_id = None
            best_mapped_id = None

            if result.boxes is not None and len(result.boxes) > 0:
                for b in result.boxes:
                    raw_id = int(b.cls[0])
                    mapped_id = remap_id(raw_id)
                    conf = float(b.conf[0])

                    # keep only weeds (your dataset meaning)
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

                # Aim point based on mapped meaning
                if best_mapped_id == 1:  # weed_side
                    cy = int(y2 - SOIL_OFFSET_FACTOR * box_h)
                else:  # weed_top
                    cy = int(y1 + TOP_Y_FACTOR * box_h)

                cy = max(0, min(cy, h - 1))
                cv2.circle(annotated, (cx, cy), 7, (0, 0, 255), -1)

                # Fire logic
                dx = abs(cx - center_x)
                dy = abs(cy - center_y)
                fire_allowed = (dx < fire_zone_w and dy < fire_zone_h)

                status = "FIRE" if fire_allowed else "NO FIRE"
                color = (0, 0, 255) if fire_allowed else (0, 255, 0)

                # Clean main text only
                cv2.putText(
                    annotated,
                    f"{status} ({mapped_name}) {best_conf:.2f}",
                    (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    color,
                    3
                )

                writer.writerow([
                    time.time(),
                    best_raw_id, best_mapped_id, mapped_name,
                    best_conf,
                    cx, cy,
                    int(fire_allowed),
                    x1, y1, x2, y2,
                    int(flip_1_2)
                ])

            else:
                cv2.putText(
                    annotated,
                    "NO WEED",
                    (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 255),
                    3
                )

            cv2.imshow("YOLO Weed Detection", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("f"):
                flip_1_2 = not flip_1_2

    cap.release()
    cv2.destroyAllWindows()
    print("Finished. CSV saved to:", LOG_FILE)


if __name__ == "__main__":
    run_realtime_detection()
