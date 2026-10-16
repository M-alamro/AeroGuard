from final_project.tracking.track_manager import (
    TrackManager,
)


manager = TrackManager()


# --------------------------------------------------
# Create two tracks
# --------------------------------------------------

manager.update(
    1,
    100,
    200,
    0.70,
)

manager.update(
    5,
    300,
    400,
    0.80,
)


print("\n=== INITIAL TRACKS ===")

print(
    manager.get_active_track_ids()
)


# --------------------------------------------------
# Frame 2
# Track 5 disappears
# --------------------------------------------------

current_ids = [1]

missing_ids = (
    manager.get_missing_track_ids(
        current_ids
    )
)


print("\n=== FRAME 2 ===")

print(
    f"Missing IDs: {missing_ids}"
)


for track_id in missing_ids:

    state = manager.predict_missing(
        track_id
    )

    print(
        f"ID={track_id} "
        f"prediction=("
        f"{state['predicted_x']:.2f}, "
        f"{state['predicted_y']:.2f}) "
        f"missed={state['missed']}"
    )


# --------------------------------------------------
# Frame 3
# Track 5 still missing
# --------------------------------------------------

current_ids = [1]

missing_ids = (
    manager.get_missing_track_ids(
        current_ids
    )
)


print("\n=== FRAME 3 ===")

print(
    f"Missing IDs: {missing_ids}"
)


for track_id in missing_ids:

    state = manager.predict_missing(
        track_id
    )

    print(
        f"ID={track_id} "
        f"prediction=("
        f"{state['predicted_x']:.2f}, "
        f"{state['predicted_y']:.2f}) "
        f"missed={state['missed']}"
    )
