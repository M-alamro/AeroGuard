import numpy as np

from .kalman import KalmanTracker


class TrackManager:

    def __init__(
        self,
        history_size=10,
        temporal_window_size=6,
        temporal_good_required=5,
        quality_confirm=0.45,
        quality_demote=0.45,
        min_current_confidence_confirm=0.35,
        min_average_confidence_confirm=0.35,
    ):
        self.max_track_missed_frames = 45

        self.history_size = history_size

        self.temporal_window_size = (
            temporal_window_size
        )

        self.temporal_good_required = (
            temporal_good_required
        )

        self.quality_confirm = quality_confirm
        self.quality_demote = quality_demote

        self.min_current_confidence_confirm = (
            min_current_confidence_confirm
        )

        self.min_average_confidence_confirm = (
            min_average_confidence_confirm
        )

        self.tracks = {}

    @staticmethod
    def calculate_quality(
        confidence,
        average_confidence,
        confidence_std,
        age,
    ):
        """
        Calculate track quality.

        V4.6 weighting:
            45% current confidence
            35% average confidence
            15% consistency
            5% age
        """

        confidence_score = float(
            confidence
        )

        age_score = min(
            age / 20.0,
            1.0,
        )

        consistency_score = max(
            0.0,
            1.0 - confidence_std / 0.20,
        )

        quality_score = (
            0.45 * confidence_score
            + 0.35 * average_confidence
            + 0.15 * consistency_score
            + 0.05 * age_score
        )

        return float(quality_score)

    def is_good_for_confirmation(
        self,
        confidence,
        average_confidence,
        quality_score,
    ):
        """
        Check whether the current observation
        is good enough for confirmation.
        """

        return (
            quality_score
            >= self.quality_confirm
            and confidence
            >= self.min_current_confidence_confirm
            and average_confidence
            >= self.min_average_confidence_confirm
        )

    def _create_track(
        self,
        track_id,
        x,
        y,
    ):
        """
        Create internal state for a new track.
        """

        self.tracks[track_id] = {
            "track_age": 0,
            "missed": 0,
            "status": "CANDIDATE",

            "confidence_history": [],
            "good_history": [],
            "confirmation_count": 0,

            "kalman": KalmanTracker(x, y),

            "last_center": (
                float(x),
                float(y),
            ),

            "identity_id": None,
        }

    def update(
        self,
        track_id,
        x,
        y,
        confidence,
    ):
        """
        Update one tracked object.

        Args:
            track_id: ByteTrack ID.
            x, y: detection center.
            confidence: YOLO confidence.

        Returns:
            Dictionary containing track state.
        """

        if track_id not in self.tracks:

            self._create_track(
                track_id,
                x,
                y,
            )

        track = self.tracks[track_id]

        track["track_age"] += 1
        track["missed"] = 0

        # --------------------------------------------------
        # Kalman prediction
        # --------------------------------------------------

        predicted_x, predicted_y = (
            track["kalman"].predict()
        )

        # --------------------------------------------------
        # Kalman correction
        # --------------------------------------------------

        (
            kalman_x,
            kalman_y,
            kalman_vx,
            kalman_vy,
        ) = track["kalman"].update(
            x,
            y,
        )

        kalman_speed = track[
            "kalman"
        ].speed()

        # --------------------------------------------------
        # Confidence history
        # --------------------------------------------------

        track["confidence_history"].append(
            float(confidence)
        )

        if (
            len(track["confidence_history"])
            > self.history_size
        ):
            track["confidence_history"].pop(0)

        history = track[
            "confidence_history"
        ]

        average_confidence = float(
            np.mean(history)
        )

        confidence_std = float(
            np.std(history)
        )

        # --------------------------------------------------
        # Quality
        # --------------------------------------------------

        quality_score = self.calculate_quality(
            confidence,
            average_confidence,
            confidence_std,
            track["track_age"],
        )

        good = self.is_good_for_confirmation(
            confidence,
            average_confidence,
            quality_score,
        )

        # --------------------------------------------------
        # Temporal confirmation
        # --------------------------------------------------

        track["good_history"].append(
            bool(good)
        )

        if (
            len(track["good_history"])
            > self.temporal_window_size
        ):
            track["good_history"].pop(0)

        good_count = sum(
            track["good_history"]
        )

        track["confirmation_count"] = (
            good_count
        )

        temporal_confirmation = (
            len(track["good_history"])
            >= self.temporal_window_size
            and good_count
            >= self.temporal_good_required
        )

        # --------------------------------------------------
        # Motion
        # --------------------------------------------------

        current_center = (
            float(x),
            float(y),
        )

        previous_center = track[
            "last_center"
        ]

        pixel_distance = float(
            np.hypot(
                current_center[0]
                - previous_center[0],
                current_center[1]
                - previous_center[1],
            )
        )

        track["last_center"] = (
            current_center
        )

        # --------------------------------------------------
        # Return complete state
        # --------------------------------------------------

        return {
            "track_id": track_id,

            "status": track["status"],

            "track_age": track["track_age"],

            "missed": track["missed"],

            "center_x": float(x),
            "center_y": float(y),

            "predicted_x": predicted_x,
            "predicted_y": predicted_y,

            "kalman_x": kalman_x,
            "kalman_y": kalman_y,

            "kalman_vx": kalman_vx,
            "kalman_vy": kalman_vy,

            "kalman_speed": kalman_speed,

            "pixel_distance": pixel_distance,

            "confidence": float(confidence),

            "average_confidence":
                average_confidence,

            "confidence_std":
                confidence_std,

            "quality_score":
                quality_score,

            "good":
                good,

            "confirmation_count":
                good_count,

            "temporal_confirmation":
                temporal_confirmation,

            "identity_id":
                track["identity_id"],
        }


    def predict_missing(self, track_id):
        """
        Predict the track position when no detection
        is available for the current frame.

        No new confidence value is added because
        YOLO did not provide a detection.
        """

        if track_id not in self.tracks:
            return None

        track = self.tracks[track_id]

        # Increase age and missed-frame counter
        track["track_age"] += 1
        track["missed"] += 1

        # Kalman prediction
        predicted_x, predicted_y = (
            track["kalman"].predict()
        )

        # Get current Kalman state
        (
            kalman_x,
            kalman_y,
            kalman_vx,
            kalman_vy,
        ) = track["kalman"].state()

        kalman_speed = (
            track["kalman"].speed()
        )

        history = track[
            "confidence_history"
        ]

        average_confidence = (
            float(np.mean(history))
            if history
            else 0.0
        )

        confidence_std = (
            float(np.std(history))
            if history
            else 0.0
        )

        return {
            "track_id": track_id,

            "status": track["status"],

            "track_age": track["track_age"],

            "missed": track["missed"],

            # No YOLO detection this frame
            "center_x": None,
            "center_y": None,

            # Kalman prediction
            "predicted_x": predicted_x,
            "predicted_y": predicted_y,

            "kalman_x": kalman_x,
            "kalman_y": kalman_y,

            "kalman_vx": kalman_vx,
            "kalman_vy": kalman_vy,

            "kalman_speed": kalman_speed,

            "confidence": None,

            "average_confidence":
                average_confidence,

            "confidence_std":
                confidence_std,

            "quality_score": None,

            "good": False,

            "confirmation_count":
                track["confirmation_count"],

            "temporal_confirmation": (
                len(track["good_history"])
                >= self.temporal_window_size
                and
                sum(track["good_history"])
                >= self.temporal_good_required
            ),

            "identity_id":
                track["identity_id"],
        }

    def get_active_track_ids(self):
        """
        Return all ByteTrack IDs currently known
        by the TrackManager.
        """

        return list(self.tracks.keys())

    def get_missing_track_ids(self, current_track_ids):
        """
        Return ByteTrack IDs that were previously known
        but are missing from the current frame.
        """

        current_track_ids = set(
            current_track_ids
        )

        known_track_ids = set(
            self.tracks.keys()
        )

        return list(
            known_track_ids
            - current_track_ids
        )

    def expire_old_tracks(self):
        """
        Expire internal ByteTrack states that have been
        missing for too many frames.

        This does NOT expire the physical identity.
        """

        expired_track_ids = []

        for track_id, track in self.tracks.items():

            # Already expired
            if track["status"] == "EXPIRED":
                continue

            if track["missed"] > self.max_track_missed_frames:

                track["status"] = "EXPIRED"

                expired_track_ids.append(track_id)

        return expired_track_ids


    def get_track(
        self,
        track_id,
    ):
        return self.tracks.get(
            track_id
        )

    def remove_track(self, track_id):
        """
        Remove an old ByteTrack state.

        This removes only the internal ByteTrack track.
        It does NOT remove the physical identity.
        """

        if track_id in self.tracks:
            del self.tracks[track_id]
            return True

        return False