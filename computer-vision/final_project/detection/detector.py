from ultralytics import YOLO


class DroneDetector:
    """
    YOLO-based drone detector.

    مسؤول عن:
    - تحميل نموذج YOLO
    - تشغيل inference
    - إرجاع نتائج الكشف
    """

    def __init__(
        self,
        model_path,
        img_size=640,
        confidence=0.25,
        device=0
    ):
        self.model_path = model_path
        self.img_size = img_size
        self.confidence = confidence
        self.device = device

        print(f"Loading YOLO model: {model_path}")

        self.model = YOLO(model_path)

        print("YOLO model loaded successfully.")

    def predict(self, frame):
        """
        Run YOLO detection on one frame.

        Args:
            frame: OpenCV image/frame.

        Returns:
            YOLO Results object.
        """

        results = self.model(
            frame,
            imgsz=self.img_size,
            conf=self.confidence,
            device=self.device,
            verbose=False
        )

        return results[0]
