import cv2

from detection.detector import DroneDetector


VIDEO_PATH = "/home/mohammed/drone_cv/test_videos/youtube/drone_test_2_h264.mp4"

MODEL_PATH = "/home/mohammed/drone_cv/runs/yolov8n_30epochs_safe/weights/best.pt"

TARGET_FRAME = 212


def main():

    detector = DroneDetector(
        model_path=MODEL_PATH,
        img_size=640,
        confidence=0.25,
        device=0
    )

    cap = cv2.VideoCapture(VIDEO_PATH)

    if not cap.isOpened():
        raise RuntimeError("Could not open video.")

    for frame_number in range(1, TARGET_FRAME + 1):

        ret, frame = cap.read()

        if not ret:
            raise RuntimeError(
                f"Could not read frame {frame_number}"
            )

    cap.release()

    result = detector.predict(frame)

    print()
    print("=" * 50)
    print(f"Frame: {TARGET_FRAME}")
    print("=" * 50)

    if result.boxes is None or len(result.boxes) == 0:

        print("No detections.")

        return

    for index, box in enumerate(result.boxes, start=1):

        confidence = float(
            box.conf[0].cpu().item()
        )

        x1, y1, x2, y2 = (
            box.xyxy[0]
            .cpu()
            .numpy()
        )

        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2

        print(
            f"Detection {index}: "
            f"confidence={confidence:.3f}, "
            f"center=({center_x:.1f}, {center_y:.1f})"
        )

    print()
    print("Detector test completed successfully.")


if __name__ == "__main__":
    main()
