from ultralytics import YOLO

from final_project.tracking.bytetrack_adapter import (
    ByteTrackAdapter,
)

from final_project.tracking.track_manager import (
    TrackManager,
)

from final_project.tracking.identity_manager import (
    IdentityManager,
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


# --------------------------------------------------
# Initialize components
# --------------------------------------------------

model = YOLO(MODEL_PATH)

adapter = ByteTrackAdapter()

track_manager = TrackManager()

identity_manager = IdentityManager()


# --------------------------------------------------
# Run YOLO + ByteTrack
# --------------------------------------------------

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


print("\n=== FULL PIPELINE TEST ===\n")


for frame_number, result in enumerate(
    results,
    start=1,
):

    tracks = adapter.extract_tracks(
        result
    )

    print(
        f"\nFrame {frame_number}"
    )

    for detection in tracks:

        byte_track_id = detection[
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

        # ------------------------------------------
        # TrackManager
        # ------------------------------------------

        track_state = track_manager.update(
            byte_track_id,
            x,
            y,
            confidence,
        )

        # ------------------------------------------
        # Only confirmed tracks can create
        # physical identities.
        # ------------------------------------------

        identity_id = None

        if track_state[
            "temporal_confirmation"
        ]:

            existing_track = (
                track_manager.get_track(
                    byte_track_id
                )
            )

            if (
                existing_track is not None
                and existing_track[
                    "identity_id"
                ] is not None
            ):

                identity_id = (
                    existing_track[
                        "identity_id"
                    ]
                )

            else:

                # ----------------------------------
                # First try LOST identity
                # reassociation.
                # ----------------------------------

                identity_id, distance = (
                    identity_manager.find_lost_identity(
                        center=(x, y),
                        confidence=confidence,
                        current_byte_track_id=
                            byte_track_id,
                    )
                )

                if identity_id is not None:

                    identity_manager.confirm_reassociation(
                        identity_id,
                        byte_track_id,
                    )

                    identity_manager.update(
                        identity_id,
                        byte_track_id,
                        (x, y),
                    )

                else:

                    # ----------------------------------
                    # Create new physical identity.
                    # ----------------------------------

                    identity_id = (
                        identity_manager.create_identity(
                            byte_track_id,
                            (x, y),
                        )
                    )

                existing_track[
                    "identity_id"
                ] = identity_id

        # ------------------------------------------
        # Update existing physical identity
        # ------------------------------------------

        if identity_id is not None:

            identity_manager.update(
                identity_id,
                byte_track_id,
                (
                    track_state[
                        "kalman_x"
                    ],
                    track_state[
                        "kalman_y"
                    ],
                ),
            )

        # ------------------------------------------
        # Print state
        # ------------------------------------------

        print(
            f"  "
            f"BT={byte_track_id} "
            f"conf={confidence:.3f} "
            f"quality="
            f"{track_state['quality_score']:.3f} "
            f"temporal="
            f"{track_state['temporal_confirmation']} "
            f"identity="
            f"{identity_id}"
        )

    # ----------------------------------------------
    # Stop after 30 frames
    # ----------------------------------------------

    if frame_number >= 30:
        break


# --------------------------------------------------
# Final summary
# --------------------------------------------------

print("\n=== FINAL SUMMARY ===")

print(
    f"Active identities: "
    f"{identity_manager.active_count()}"
)

print(
    f"Lost identities: "
    f"{identity_manager.lost_count()}"
)

print(
    f"Total identities created: "
    f"{len(identity_manager.identities)}"
)

for identity_id, identity in (
    identity_manager.identities.items()
):

    print(
        f"Identity {identity_id}: "
        f"status={identity['status']} "
        f"last_BT="
        f"{identity['last_byte_track_id']} "
        f"reassociations="
        f"{identity['reassociations']}"
    )
