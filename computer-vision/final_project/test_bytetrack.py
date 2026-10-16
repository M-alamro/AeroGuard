from ultralytics import YOLO


MODEL_PATH = (
    "/home/mohammed/drone_cv/"
    "runs/yolov8n_30epochs_safe/"
    "weights/best.pt"
)

VIDEO_PATH = (
    "/home/mohammed/drone_cv/"
    "test_videos/youtube/"
    "drone_test_2_h264.mp4"
)


model = YOLO(MODEL_PATH)

results = model.track(
    source=VIDEO_PATH,
    tracker="bytetrack.yaml",
    imgsz=640,
    conf=0.25,
    device=0,
    persist=True,
    stream=True,
    verbose=False,
)


print("\n=== BYTE TRACK TEST ===\n")


for frame_number, result in enumerate(
    results,
    start=1
):

    boxes = result.boxes

    if boxes is None or len(boxes) == 0:
        print(
            f"Frame {frame_number}: "
            "No detections"
        )
        continue

    ids = boxes.id

    if ids is None:
        print(
            f"Frame {frame_number}: "
            "Detection without Track ID"
        )
        continue

    ids = ids.cpu().numpy().astype(int)

    confidences = (
        boxes.conf.cpu().numpy()
    )

    print(
        f"Frame {frame_number}: "
        f"IDs={ids.tolist()} "
        f"Conf={confidences.round(3).tolist()}"
    )

    if frame_number >= 30:
        break
