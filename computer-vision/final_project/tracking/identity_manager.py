import numpy as np


class IdentityManager:
    """
    Manages persistent physical drone identities.

    Lifecycle:

        Candidate
            ↓
        Confirmed
            ↓
          Lost
            ↓
        Re-associated
            ↓
        Confirmed

    A new physical identity is created only after
    temporal confirmation.
    """

    def __init__(
        self,
        # max_missed_frames=45,
        max_missed_frames=150,
        reassociation_max_distance=120.0,
        reassociation_min_confidence=0.35,
        early_reassociation_max_distance=100.0,
        early_reassociation_min_confidence=0.45,
        early_reassociation_confirmation_frames=3,
    ):

        self.max_missed_frames = max_missed_frames

        self.reassociation_max_distance = (
            reassociation_max_distance
        )

        self.reassociation_min_confidence = (
            reassociation_min_confidence
        )

        self.early_reassociation_max_distance = (
            early_reassociation_max_distance
        )

        self.early_reassociation_min_confidence = (
            early_reassociation_min_confidence
        )

        self.early_reassociation_confirmation_frames = (
            early_reassociation_confirmation_frames
        )

        self.identities = {}

        self.next_identity_id = 1

        self.early_reassociation_candidates = {}

    @staticmethod
    def point_distance(a, b):
        """
        Euclidean distance between two points.
        """

        if a is None or b is None:
            return float("inf")

        return float(
            np.hypot(
                a[0] - b[0],
                a[1] - b[1],
            )
        )

    def create_identity(
        self,
        byte_track_id,
        prediction,
    ):
        """
        Create a new confirmed physical identity.
        """

        identity_id = self.next_identity_id

        self.next_identity_id += 1

        self.identities[identity_id] = {
            "status": "CONFIRMED",
            "age": 0,
            "missed": 0,
            "last_byte_track_id": byte_track_id,
            "prediction": prediction,
            "reassociations": 0,
        }

        return identity_id

    def find_lost_identity(
        self,
        center,
        confidence,
        current_byte_track_id,
    ):
        """
        Find the closest LOST physical identity
        that can be re-associated.
        """

        if confidence < self.reassociation_min_confidence:
            return None, float("inf")

        best_identity = None
        best_distance = float("inf")

        for identity_id, identity in self.identities.items():

            if identity["status"] != "LOST":
                continue

            if identity["missed"] > self.max_missed_frames:
                continue

            if (
                identity["last_byte_track_id"]
                == current_byte_track_id
            ):
                continue

            distance = self.point_distance(
                center,
                identity["prediction"],
            )

            if (
                distance
                <= self.reassociation_max_distance
                and distance < best_distance
            ):
                best_identity = identity_id
                best_distance = distance

        return best_identity, best_distance

    def find_early_reassociation(
        self,
        center,
        confidence,
    ):
        """
        Find a LOST identity for early recovery.
        """

        if (
            confidence
            < self.early_reassociation_min_confidence
        ):
            return None, float("inf")

        best_identity = None
        best_distance = float("inf")

        for identity_id, identity in self.identities.items():

            if identity["status"] != "LOST":
                continue

            if identity["missed"] > self.max_missed_frames:
                continue

            distance = self.point_distance(
                center,
                identity["prediction"],
            )

            if (
                distance
                <= self.early_reassociation_max_distance
                and distance < best_distance
            ):
                best_identity = identity_id
                best_distance = distance

        return best_identity, best_distance


    def update_early_reassociation_candidate(
        self,
        identity_id,
        byte_track_id,
        good_for_confirmation,
    ):
        """
        Register one observation for an early
        reassociation candidate.

        Only good observations increase the
        confirmation counter.

        Returns:
            confirmation_count
            confirmed
        """

        key = (
            identity_id,
            byte_track_id,
        )

        if key not in self.early_reassociation_candidates:
            self.early_reassociation_candidates[key] = 0

        if good_for_confirmation:
            self.early_reassociation_candidates[key] += 1
        else:
            self.early_reassociation_candidates[key] = 0

        count = (
            self.early_reassociation_candidates[key]
        )

        confirmed = (
            count
            >= self.early_reassociation_confirmation_frames
        )

        return count, confirmed

    def clear_early_reassociation_candidate(
        self,
        identity_id,
        byte_track_id,
    ):
        """
        Remove an early reassociation candidate.
        """

        key = (
            identity_id,
            byte_track_id,
        )

        self.early_reassociation_candidates.pop(
            key,
            None,
        )

    def mark_lost(
        self,
        identity_id,
        prediction,
    ):
        """
        Mark a physical identity as LOST.
        """

        if identity_id is None:
            return

        if identity_id not in self.identities:
            return

        identity = self.identities[identity_id]

        identity["status"] = "LOST"

        if identity["missed"] == 0:
            identity["missed"] = 1
        else:
            identity["missed"] += 1

        identity["prediction"] = prediction

    def update(
        self,
        identity_id,
        byte_track_id,
        prediction,
    ):
        """
        Update an active physical identity.
        """

        if identity_id is None:
            return

        if identity_id not in self.identities:
            return

        identity = self.identities[identity_id]

        identity["age"] += 1
        identity["missed"] = 0
        identity["status"] = "CONFIRMED"

        identity["last_byte_track_id"] = (
            byte_track_id
        )

        identity["prediction"] = prediction

    def confirm_reassociation(
        self,
        identity_id,
        byte_track_id,
    ):
        """
        Confirm a successful reassociation.
        """

        if identity_id is None:
            return

        if identity_id not in self.identities:
            return

        identity = self.identities[identity_id]

        identity["status"] = "CONFIRMED"
        identity["missed"] = 0

        identity["last_byte_track_id"] = (
            byte_track_id
        )

        identity["reassociations"] += 1


    def expire_old_identities(self):
        """
        Expire LOST identities that exceeded
        the maximum allowed missed frames.

        Returns:
            List of identities that changed
            from LOST to EXPIRED.
        """

        expired_ids = []

        for identity_id, identity in self.identities.items():

            # Only LOST identities can expire.
            if identity["status"] != "LOST":
                continue

            if identity["missed"] > self.max_missed_frames:

                identity["status"] = "EXPIRED"

                expired_ids.append(
                    identity_id
                )

        return expired_ids

    def get_identity(self, identity_id):
        """
        Return identity information.
        """

        return self.identities.get(
            identity_id
        )

    def active_count(self):
        """
        Number of confirmed physical identities.
        """

        return sum(
            1
            for identity in self.identities.values()
            if identity["status"] == "CONFIRMED"
        )

    def lost_count(self):
        """
        Number of LOST identities.
        """

        return sum(
            1
            for identity in self.identities.values()
            if identity["status"] == "LOST"
        )
