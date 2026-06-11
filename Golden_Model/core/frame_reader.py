# core/frame_reader.py
import cv2

class FrameReader:
    """
    Unified frame reader for video files or live camera.
    Usage:
      - FrameReader(video_path="myvideo.mp4")
      - FrameReader(camera_index=0)
    """

    def __init__(self, video_path: str = None, camera_index: int = None):
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open video file: {video_path}")
        elif camera_index is not None:
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open camera index: {camera_index}")
        else:
            raise ValueError("Either video_path or camera_index must be provided.")

        # probe properties
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 25.0
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

    def read(self):
        """Return (ret, frame). Frame is BGR (OpenCV)."""
        return self.cap.read()

    def release(self):
        self.cap.release()
