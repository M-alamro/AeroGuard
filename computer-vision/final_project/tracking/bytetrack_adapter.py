class ByteTrackAdapter:
    """
    Convert Ultralytics YOLO tracking results
    into a simple format for TrackManager.
    """

    def extract_tracks(self, result):
        """
        Extract tracked objects from one YOLO result.

        Returns:
            List of dictionaries:
            [
                {
                    "track_id": int,
                    "center_x": float,
                    "center_y": float,
                    "confidence": float,
                }
            ]
        """

        tracks = []

        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            return tracks

        if boxes.id is None:
            return tracks

        track_ids = (
            boxes.id
            .cpu()
            .numpy()
            .astype(int)
        )

        confidences = (
            boxes.conf
            .cpu()
            .numpy()
        )

        xyxy = (
            boxes.xyxy
            .cpu()
            .numpy()
        )

        for track_id, confidence, box in zip(
            track_ids,
            confidences,
            xyxy,
        ):

            x1, y1, x2, y2 = box

            center_x = (
                float(x1) + float(x2)
            ) / 2.0

            center_y = (
                float(y1) + float(y2)
            ) / 2.0

            tracks.append(
                {
                    "track_id": int(track_id),

                    "center_x": center_x,

                    "center_y": center_y,

                    "confidence": float(confidence),

                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                }
            )

        return tracks
