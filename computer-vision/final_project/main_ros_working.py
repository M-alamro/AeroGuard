import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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


IMAGE_TOPIC = (
    "/world/default/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)


IMG_SIZE = 640
CONFIDENCE = 0.25
DEVICE = 0


# ----------------------------------------------
# Visualization
# ----------------------------------------------

def draw_detection(frame, detection, track_state):
    """
    Draw the current drone detection and tracking information.
    """

    x1 = int(detection["x1"])
    y1 = int(detection["y1"])
    x2 = int(detection["x2"])
    y2 = int(detection["y2"])

    byte_track_id = detection["track_id"]
    confidence = detection["confidence"]

    identity_id = track_state.get("identity_id")

    # Bounding box
    cv2.rectangle(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2,
    )

    # Identity text
    identity_text = (
        f"ID:{identity_id}"
        if identity_id is not None
        else "ID:-"
    )

    label = (
        f"{identity_text} "
        f"BT:{byte_track_id} "
        f"Conf:{confidence:.2f}"
    )

    # Background for text
    (text_width, text_height), _ = cv2.getTextSize(
        label,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        1,
    )

    cv2.rectangle(
        frame,
        (x1, max(0, y1 - text_height - 8)),
        (x1 + text_width + 6, y1),
        (0, 255, 0),
        -1,
    )

    cv2.putText(
        frame,
        label,
        (x1 + 3, y1 - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )

def process_live_camera():

    print("\n=== DRONE TRACKING SYSTEM ===\n")

    # ----------------------------------------------
    # Initialize model
    # ----------------------------------------------

    print("Loading YOLO model...")

    model = YOLO(
        MODEL_PATH
    )

    print("YOLO model loaded.")


    # ----------------------------------------------
    # Initialize ROS
    # ----------------------------------------------

    rclpy.init()

    node = Node("drone_cv_node")

    bridge = CvBridge()

    # ----------------------------------------------
    # Subscribe to Gazebo camera
    # ----------------------------------------------

    latest_frame = None

    def image_callback(msg):
        nonlocal latest_frame
        latest_frame = bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

    subscription = node.create_subscription(
        Image,
        IMAGE_TOPIC,
        image_callback,
        10,
    )


    # ----------------------------------------------
    # Initialize tracking components
    # ----------------------------------------------

    adapter = ByteTrackAdapter()

    track_manager = TrackManager()

    identity_manager = IdentityManager()

    # ----------------------------------------------
    # ROS camera processing loop
    # ----------------------------------------------

    frame_number = 0

    print("\nProcessing Gazebo camera...\n")

    while rclpy.ok():

        rclpy.spin_once(
            node,
            timeout_sec=0.01,
        )

        if latest_frame is None:
            continue

        frame = latest_frame.copy()

        frame_number += 1

        results = model.track(
            source=frame,
            tracker="bytetrack.yaml",
            imgsz=IMG_SIZE,
            conf=CONFIDENCE,
            device=DEVICE,
            persist=True,
            stream=False,
            verbose=False,
        )

        result = results[0]


        # ==========================================
        # 1. Extract current ByteTrack detections
        # ==========================================

        detections = (
            adapter.extract_tracks(
                result
            )
        )

        current_track_ids = [
            detection["track_id"]
            for detection in detections
        ]

        # ==========================================
        # 2. Process CURRENT detections first
        # ==========================================

        for detection in detections:

            byte_track_id = (
                detection["track_id"]
            )

            x = detection[
                "center_x"
            ]

            y = detection[
                "center_y"
            ]

            confidence = detection[
                "confidence"
            ]

            # --------------------------------------
            # Update TrackManager
            # --------------------------------------

            track_state = (
                track_manager.update(
                    byte_track_id,
                    x,
                    y,
                    confidence,
                )
            )

            track = (
                track_manager.get_track(
                    byte_track_id
                )
            )

            if track is None:
                continue

            identity_id = (
                track["identity_id"]
            )
                        # --------------------------------------
            # Early reassociation
            # --------------------------------------

            # --------------------------------------
            # Early reassociation
            # --------------------------------------

            if identity_id is None:

                early_identity, early_distance = (
                    identity_manager.find_early_reassociation(
                        center=(x, y),
                        confidence=confidence,
                    )
                )

                if early_identity is not None:

                    count, confirmed = (
                        identity_manager
                        .update_early_reassociation_candidate(
                            identity_id=early_identity,
                            byte_track_id=byte_track_id,
                        )
                    )

                    print(
                        f"[EARLY CANDIDATE] "
                        f"Frame={frame_number} "
                        f"BT={byte_track_id} "
                        f"Identity={early_identity} "
                        f"G={count}/3 "
                        f"distance={early_distance:.2f}"
                    )

                    if confirmed:

                        old_byte_track_id = (
                            identity_manager.get_identity(
                                early_identity
                            )["last_byte_track_id"]
                        )

                        identity_id = early_identity

                        identity_manager.confirm_reassociation(
                            identity_id,
                            byte_track_id,
                        )

                        track["identity_id"] = identity_id

                        identity_manager.clear_early_reassociation_candidate(
                            identity_id,
                            byte_track_id,
                        )

                        if (
                            old_byte_track_id is not None
                            and old_byte_track_id != byte_track_id
                        ):
                            track_manager.remove_track(
                                old_byte_track_id
                            )

                        print(
                            f"\n[EARLY REASSOCIATION CONFIRMED] "
                            f"Frame={frame_number} "
                            f"BT={byte_track_id} "
                            f"Identity={identity_id} "
                            f"OldBT={old_byte_track_id} "
                            f"distance={early_distance:.2f}\n"
                        )

            if (
                track_state[
                    "temporal_confirmation"
                ]
                and identity_id is None
            ):

                # ----------------------------------
                # First try LOST identity
                # ----------------------------------

                found_identity, distance = (
                    identity_manager.find_lost_identity(
                        center=(x, y),
                        confidence=confidence,
                        current_byte_track_id=
                            byte_track_id,
                    )
                )

                if found_identity is not None:

                    identity_id = (
                        found_identity
                    )

                    identity_manager.confirm_reassociation(
                        identity_id,
                        byte_track_id,
                    )

                    print(
                        f"\n[REASSOCIATION] "
                        f"Frame={frame_number} "
                        f"BT={byte_track_id} "
                        f"Identity={identity_id} "
                        f"distance={distance:.2f}\n"
                    )

                # ----------------------------------
                # Otherwise create new identity
                # ----------------------------------

                else:

                    identity_id = (
                        identity_manager.create_identity(
                            byte_track_id,
                            (
                                track_state["kalman_x"],
                                track_state["kalman_y"],
                            ),
                        )
                    )

                    print(
                        f"\n[NEW IDENTITY] "
                        f"Frame={frame_number} "
                        f"BT={byte_track_id} "
                        f"Identity={identity_id}\n"
                    )

                track[
                    "identity_id"
                ] = identity_id

            # --------------------------------------
            # Update confirmed identity
            # --------------------------------------

            if identity_id is not None:

                identity = (
                    identity_manager.get_identity(
                        identity_id
                    )
                )

                # IMPORTANT:
                # Do not revive an EXPIRED identity.
                if (
                    identity is not None
                    and identity["status"]
                    != "EXPIRED"
                ):

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

            # --------------------------------------
            # Terminal output
            # --------------------------------------

            identity_text = (
                str(identity_id)
                if identity_id is not None
                else "-"
            )

            print(
                f"Frame={frame_number:03d} "
                f"BT={byte_track_id:02d} "
                f"ID={identity_text:>2} "
                f"conf={confidence:.3f} "
                f"Q={track_state['quality_score']:.3f} "
                f"temporal="
                f"{track_state['temporal_confirmation']} "
                f"speed="
                f"{track_state['kalman_speed']:.2f} "
                f"missed="
                f"{track_state['missed']}"
            )

            # --------------------------------------
            # Visualization
            # --------------------------------------
            draw_detection(
                frame=result.orig_img,
                detection=detection,
                track_state=track_state,
            )

        # ----------------------------------------------
        # Display annotated frame
        # ----------------------------------------------


        cv2.imshow(
            "Drone CV - Gazebo",
            result.orig_img,
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        # ==========================================
        # 3. NOW process missing ByteTrack tracks
        # ==========================================

        missing_track_ids = (
            track_manager.get_missing_track_ids(
                current_track_ids
            )
        )

        for track_id in missing_track_ids:

            missing_state = (
                track_manager.predict_missing(
                    track_id
                )
            )

            if missing_state is None:
                continue

            track = (
                track_manager.get_track(
                    track_id
                )
            )

            if track is None:
                continue

            identity_id = (
                track["identity_id"]
            )

            if identity_id is None:
                continue

            identity = (
                identity_manager.get_identity(
                    identity_id
                )
            )

            if identity is None:
                continue

            # --------------------------------------
            # Never revive/re-process EXPIRED ID
            # --------------------------------------

            if identity["status"] == "EXPIRED":
                continue

            # --------------------------------------
            # Mark physical identity as LOST
            # --------------------------------------

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

        # ==========================================
        # 4. Expire old ByteTrack states
        # ==========================================

        expired_track_ids = (
            track_manager.expire_old_tracks()
        )

        for track_id in expired_track_ids:

            print(
                f"\n[TRACK EXPIRED] "
                f"BT={track_id} "
                f"at Frame={frame_number}\n"
            )

        # ==========================================
        # 5. Expire LOST identities
        # ==========================================

        expired_ids = (
            identity_manager
            .expire_old_identities()
        )

    # ----------------------------------------------
    # Shutdown
    # ----------------------------------------------

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()

    # ==============================================
    # Final summary
    # ==============================================

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
        f"Total identities: "
        f"{len(identity_manager.identities)}"
    )


if __name__ == "__main__":
    process_live_camera()