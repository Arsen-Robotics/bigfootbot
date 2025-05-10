import asyncio
import fractions
import logging
import time
from typing import Optional

import av
import cv2
from aiortc.mediastreams import MediaStreamTrack

logger = logging.getLogger("video_server")

class CameraVideoTrack(MediaStreamTrack):
    """
    A video track that captures from a camera device.
    """

    kind = "video"

    def __init__(self, camera_id=0):
        """
        Initialize the camera video track.
        
        Args:
            camera_id: Camera device index (default: 0 for primary camera)
        """
        super().__init__()
        self.camera_id = camera_id
        self.camera = None
        self.cap_width = 640  # default width
        self.cap_height = 480  # default height
        self.cap_fps = 30  # default fps
        self._frame_counter = 0
        self._start_time = time.time()
        self._last_log = self._start_time
        
        try:
            # Open the camera
            self.camera = cv2.VideoCapture(camera_id)
            
            # Try to set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.cap_fps)
            
            # Get actual properties
            self.cap_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.cap_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.cap_fps = int(self.camera.get(cv2.CAP_PROP_FPS))
            
            logger.info(f"Camera {camera_id} opened with resolution {self.cap_width}x{self.cap_height} @ {self.cap_fps}fps")
            
            # Check if camera is opened successfully
            if not self.camera.isOpened():
                raise RuntimeError(f"Could not open camera {camera_id}")
                
        except Exception as e:
            logger.error(f"Error initializing camera {camera_id}: {e}")
            self.camera = None
    
    async def next_timestamp(self):
        """Calculate the timestamp for the next frame."""
        if hasattr(self, "_timestamp"):
            self._timestamp += 1 / self.cap_fps
        else:
            self._timestamp = time.time()
        return self._timestamp
    
    async def recv(self):
        """
        Receive the next frame from the camera.
        
        Returns:
            A video frame captured from the camera.
        """
        if self.camera is None:
            # Create a black frame if camera is not available
            pts, time_base = await self._next_timestamp()
            img = self._create_black_frame()
            
            # Create a video frame from the image
            frame = av.VideoFrame.from_ndarray(img, format="bgr24")
            frame.pts = pts
            frame.time_base = time_base
            
            return frame
        
        # Read frame from camera
        ret, img = self.camera.read()
        
        # If frame was not read correctly, create a black frame
        if not ret:
            logger.warning(f"Failed to read from camera {self.camera_id}")
            img = self._create_black_frame()
        
        # Get the timestamp for this frame
        pts, time_base = await self._next_timestamp()
        
        # Log FPS every few seconds
        self._frame_counter += 1
        now = time.time()
        if now - self._last_log > 5.0:  # Log every 5 seconds
            elapsed = now - self._last_log
            fps = self._frame_counter / elapsed
            logger.debug(f"Camera {self.camera_id} capturing at {fps:.2f} FPS")
            self._frame_counter = 0
            self._last_log = now
        
        # Convert the image to a video frame
        frame = av.VideoFrame.from_ndarray(img, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        
        return frame
    
    async def _next_timestamp(self):
        """
        Calculate the next timestamp for the frame.
        
        Returns:
            Tuple of (pts, time_base)
        """
        pts = int(await self.next_timestamp() * 90000)
        time_base = fractions.Fraction(1, 90000)
        return pts, time_base
    
    def _create_black_frame(self):
        """
        Create a black frame with the configured dimensions.
        
        Returns:
            A black frame as a numpy array.
        """
        return cv2.rectangle(
            cv2.rectangle(
                cv2.rectangle(
                    cv2.rectangle(
                        cv2.putText(
                            cv2.putText(
                                cv2.putText(
                                    # Create a blank black image
                                    cv2.cvtColor(
                                        cv2.cvtColor(
                                            # Start with a black image
                                            cv2.UMat(self.cap_height, self.cap_width, cv2.CV_8UC1),
                                            cv2.COLOR_GRAY2BGR
                                        ),
                                        cv2.COLOR_BGR2RGB
                                    ).get(),
                                    "Camera Not Available",
                                    (int(self.cap_width / 2) - 125, int(self.cap_height / 2) - 50),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8,
                                    (255, 255, 255),
                                    2,
                                    cv2.LINE_AA
                                ),
                                f"Camera ID: {self.camera_id}",
                                (int(self.cap_width / 2) - 100, int(self.cap_height / 2)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (200, 200, 200),
                                1,
                                cv2.LINE_AA
                            ),
                            "Please check camera connection",
                            (int(self.cap_width / 2) - 130, int(self.cap_height / 2) + 50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (200, 200, 200),
                            1,
                            cv2.LINE_AA
                        ),
                        0, 0, self.cap_width, self.cap_height,
                        (50, 50, 50), 2
                    ),
                    5, 5, self.cap_width - 10, self.cap_height - 10,
                    (100, 100, 100), 1
                ),
                10, 10, self.cap_width - 20, self.cap_height - 20,
                (50, 50, 50), 1
            ),
            15, 15, self.cap_width - 30, self.cap_height - 30,
            (30, 30, 30), 1
        )
    
    def __del__(self):
        """Clean up resources when the track is no longer used."""
        if self.camera is not None:
            self.camera.release()
            logger.info(f"Camera {self.camera_id} released") 