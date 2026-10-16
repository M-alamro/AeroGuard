import cv2
import numpy as np


class KalmanTracker:
    """
    Kalman Filter for 2D object tracking.

    State:
        [x, y, vx, vy]

    Measurement:
        [x, y]
    """

    def __init__(self, x, y):

        self.kalman = cv2.KalmanFilter(4, 2)

        # State transition matrix
        self.kalman.transitionMatrix = np.array(
            [
                [1, 0, 1, 0],
                [0, 1, 0, 1],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

        # Measurement matrix
        self.kalman.measurementMatrix = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            dtype=np.float32,
        )

        # Process noise
        self.kalman.processNoiseCov = np.array(
            [
                [1e-2, 0, 0, 0],
                [0, 1e-2, 0, 0],
                [0, 0, 5e-2, 0],
                [0, 0, 0, 5e-2],
            ],
            dtype=np.float32,
        )

        # Measurement noise
        self.kalman.measurementNoiseCov = np.array(
            [
                [1e-1, 0],
                [0, 1e-1],
            ],
            dtype=np.float32,
        )

        self.kalman.errorCovPost = np.eye(
            4,
            dtype=np.float32
        )

        # Initial state
        self.kalman.statePost = np.array(
            [
                [x],
                [y],
                [0.0],
                [0.0],
            ],
            dtype=np.float32,
        )

    def predict(self):
        """
        Predict the next position.

        Returns:
            (x, y)
        """

        prediction = self.kalman.predict()

        x = float(prediction[0, 0])
        y = float(prediction[1, 0])

        return x, y

    def update(self, x, y):
        """
        Correct the Kalman state using a new detection.

        Args:
            x: Detected center X.
            y: Detected center Y.

        Returns:
            x, y, vx, vy
        """

        measurement = np.array(
            [
                [np.float32(x)],
                [np.float32(y)],
            ]
        )

        corrected = self.kalman.correct(
            measurement
        )

        corrected_x = float(corrected[0, 0])
        corrected_y = float(corrected[1, 0])

        velocity_x = float(corrected[2, 0])
        velocity_y = float(corrected[3, 0])

        return (
            corrected_x,
            corrected_y,
            velocity_x,
            velocity_y,
        )

    def speed(self):
        """
        Return current speed in pixels/frame.
        """

        vx = float(
            self.kalman.statePost[2, 0]
        )

        vy = float(
            self.kalman.statePost[3, 0]
        )

        return float(
            np.hypot(vx, vy)
        )

    def state(self):
        """
        Return the current Kalman state.

        Returns:
            x, y, vx, vy
        """

        x = float(self.kalman.statePost[0, 0])
        y = float(self.kalman.statePost[1, 0])
        vx = float(self.kalman.statePost[2, 0])
        vy = float(self.kalman.statePost[3, 0])

        return x, y, vx, vy