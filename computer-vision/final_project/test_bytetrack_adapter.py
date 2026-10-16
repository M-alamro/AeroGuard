from ultralytics import YOLO

from final_project.tracking.bytetrack_adapter import (
    ByteTrackAdapter,
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


print("\n=== BYTE TRACK ADAPTER TEST ===\n")


for frame_number, result in enumerate(
    results,
    start=1,
):

    tracks = adapter.extract_tracks(
        result
    )

    print(
        f"Frame {frame_number}: "
        f"{len(tracks)} track(s)"
    )

    for track in tracks:

        print(
            f"  ID={track['track_id']} "
            f"center=("
            f"{track['center_x']:.2f}, "
            f"{track['center_y']:.2f}) "
            f"confidence="
            f"{track['confidence']:.3f}"
        )

    if frame_number >= 10:
        break
