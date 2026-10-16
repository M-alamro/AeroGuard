from final_project.tracking.track_manager import TrackManager


manager = TrackManager()


print("\n=== DETECTIONS ===")

detections = [
    (100, 200, 0.70),
    (105, 203, 0.72),
    (110, 206, 0.68),
    (115, 209, 0.75),
]

for frame, (x, y, confidence) in enumerate(
    detections,
    start=1
):

    result = manager.update(
        1,
        x,
        y,
        confidence,
    )

    print(
        f"Frame {frame}: "
        f"position=({result['kalman_x']:.2f}, "
        f"{result['kalman_y']:.2f}) "
        f"speed={result['kalman_speed']:.2f}"
    )


print("\n=== MISSING DETECTIONS ===")

for frame in range(5, 8):

    result = manager.predict_missing(1)

    print(
        f"Frame {frame}: "
        f"prediction=({result['predicted_x']:.2f}, "
        f"{result['predicted_y']:.2f}) "
        f"speed={result['kalman_speed']:.2f} "
        f"missed={result['missed']}"
    )
