from ultralytics import YOLO

from final_project.tracking.bytetrack_adapter import (
    ByteTrackAdapter,
)

from final_project.tracking.track_manager import (
    TrackManager,
)


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

adapter = ByteTrackAdapter()

track_manager = TrackManager()


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


print("\n=== TRACKING PIPELINE TEST ===\n")


for frame_number, result in enumerate(
    results,
    start=1,
):

    tracks = adapter.extract_tracks(
        result
    )

    print(f"Frame {frame_number}:")

    for detection in tracks:

        track_id = detection[
            "track_id"
        ]

        x = detection[
            "center_x"
        ]

        y = detection[
            "center_y"
        ]

        confidence = detection[
            "confidence"
        ]

        state = track_manager.update(
            track_id,
            x,
            y,
            confidence,
        )

        print(
            f"  ID={track_id} "
            f"conf={confidence:.3f} "
            f"quality={state['quality_score']:.3f} "
            f"good={state['good']} "
            f"confirmation="
            f"{state['confirmation_count']}/6 "
            f"temporal="
            f"{state['temporal_confirmation']} "
            f"kalman=("
            f"{state['kalman_x']:.1f}, "
            f"{state['kalman_y']:.1f}) "
            f"speed="
            f"{state['kalman_speed']:.2f}"
        )

    if frame_number >= 10:
        break
