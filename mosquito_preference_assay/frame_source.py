"""A video file or a directory of frame images, read uniformly one frame at a
time. Shared by video_publisher_node.py and dual_video_publisher_node.py.
"""

from pathlib import Path

import cv2

FRAME_EXTS = (".bmp", ".png", ".jpg", ".jpeg")


class FrameSource:
    """Wraps either a directory of frame images (sorted by filename) or a
    video file (cv2.VideoCapture) behind next_frame() / restart()."""

    def __init__(self, source):
        path = Path(source)
        if not path.exists():
            raise RuntimeError(f"source not found: {path}")
        self.path = path

        if path.is_dir():
            self._frames = sorted(
                (f for f in path.iterdir() if f.suffix.lower() in FRAME_EXTS),
                key=lambda f: f.name,
            )
            if not self._frames:
                raise RuntimeError(f"no frame images ({FRAME_EXTS}) found in {path}")
            self._cap = None
            self._idx = 0
        else:
            self._cap = cv2.VideoCapture(str(path))
            if not self._cap.isOpened():
                raise RuntimeError(f"could not open video {path}")
            self._frames = None

    @property
    def description(self):
        if self._frames is not None:
            return f"{len(self._frames)} frames from '{self.path}'"
        return f"video '{self.path}'"

    def next_frame(self):
        """Return the next frame (a BGR or grayscale numpy array), or None
        once the source is exhausted.

        Frame-directory sources are read with IMREAD_UNCHANGED so a genuinely
        grayscale source (common for mono machine-vision cameras, e.g. Basler
        ac*m*) stays 2D instead of being upconverted to a 3-channel image and
        immediately converted back downstream -- see the ndim check in
        video_publisher_node.py / dual_video_publisher_node.py.
        """
        if self._frames is not None:
            if self._idx >= len(self._frames):
                return None
            frame = cv2.imread(str(self._frames[self._idx]), cv2.IMREAD_UNCHANGED)
            self._idx += 1
            return frame
        ok, frame = self._cap.read()
        return frame if ok else None

    def restart(self):
        """Rewind to the first frame."""
        if self._frames is not None:
            self._idx = 0
        else:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
