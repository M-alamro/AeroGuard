from final_project.tracking.track_manager import (
    TrackManager,
)

from final_project.tracking.identity_manager import (
    IdentityManager,
)


track_manager = TrackManager()

identity_manager = IdentityManager()


print("\n=== REASSOCIATION PIPELINE TEST ===\n")


# --------------------------------------------------
# Frame 1
# ByteTrack ID 1 appears
# --------------------------------------------------

state = track_manager.update(
    track_id=1,
    x=100,
    y=200,
    confidence=0.75,
)

print("Frame 1")
print(
    f"BT=1 "
    f"temporal={state['temporal_confirmation']}"
)


# --------------------------------------------------
# Frames 2-6
# Build temporal confirmation
# --------------------------------------------------

for frame in range(2, 7):

    state = track_manager.update(
        track_id=1,
        x=100 + frame * 5,
        y=200 + frame * 3,
        confidence=0.75,
    )

    print(
        f"Frame {frame}: "
        f"BT=1 "
        f"temporal="
        f"{state['temporal_confirmation']}"
    )


# --------------------------------------------------
# Create Physical Identity
# --------------------------------------------------

identity_id = (
    identity_manager.create_identity(
        byte_track_id=1,
        prediction=(
            state["kalman_x"],
            state["kalman_y"],
        ),
    )
)

track_manager.get_track(1)[
    "identity_id"
] = identity_id


print(
    f"\nCreated Identity {identity_id}"
)


# --------------------------------------------------
# Simulate ByteTrack ID 1 disappearing
# --------------------------------------------------

print("\n=== TRACK LOST ===")

missing_ids = (
    track_manager.get_missing_track_ids(
        []
    )
)

print(
    f"Missing IDs: {missing_ids}"
)


for track_id in missing_ids:

    missing_state = (
        track_manager.predict_missing(
            track_id
        )
    )

    track = track_manager.get_track(
        track_id
    )

    identity_id = track[
        "identity_id"
    ]

    if identity_id is not None:

        prediction = (
            missing_state["predicted_x"],
            missing_state["predicted_y"],
        )

        identity_manager.mark_lost(
            identity_id=identity_id,
            prediction=prediction,
            missed=missing_state[
                "missed"
            ],
        )

        print(
            f"BT={track_id} "
            f"→ Identity={identity_id} "
            f"LOST "
            f"prediction={prediction}"
        )


# --------------------------------------------------
# Simulate new ByteTrack ID
# --------------------------------------------------

new_byte_track_id = 29

new_center = (
    missing_state["predicted_x"] + 20,
    missing_state["predicted_y"] + 10,
)

new_confidence = 0.70


print("\n=== NEW BYTE TRACK ID ===")

print(
    f"BT={new_byte_track_id} "
    f"center={new_center} "
    f"confidence={new_confidence}"
)


# --------------------------------------------------
# Search for LOST physical identity
# --------------------------------------------------

found_identity, distance = (
    identity_manager.find_lost_identity(
        center=new_center,
        confidence=new_confidence,
        current_byte_track_id=
            new_byte_track_id,
    )
)


print(
    f"\nFound Identity: "
    f"{found_identity}"
)

print(
    f"Distance: "
    f"{distance:.2f}"
)


# --------------------------------------------------
# Reassociate
# --------------------------------------------------

if found_identity is not None:

    identity_manager.confirm_reassociation(
        identity_id=found_identity,
        byte_track_id=new_byte_track_id,
    )

    identity_manager.update(
        identity_id=found_identity,
        byte_track_id=new_byte_track_id,
        prediction=new_center,
    )


print("\n=== FINAL RESULT ===")

print(
    identity_manager.get_identity(
        found_identity
    )
)

print(
    f"Active identities: "
    f"{identity_manager.active_count()}"
)

print(
    f"Lost identities: "
    f"{identity_manager.lost_count()}"
)
