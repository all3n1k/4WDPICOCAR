import cv2
import time
import threading
import requests
from dimos_lite.core import Module, StreamOut


class VisionModule(Module):
    MAX_RECONNECT_DELAY = 10

    def __init__(self, stream_url="http://192.168.1.216:81/stream"):
        super().__init__("Vision")
        self.stream_url = stream_url
        self.color_image = StreamOut("color_image")
        self._base_url = stream_url.rsplit('/', 1)[0].replace(':81', '')

        # Latest-frame buffer: grab thread writes, publish thread reads
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()

    def _set_camera_params(self):
        """Push low-latency + exposure-tuned settings to ESP32-CAM."""
        params = [
            ("framesize", 6),    # SVGA 800x600
            ("quality", 15),     # Higher res needs slightly more compression for latency
            ("brightness", 0),   # Neutral brightness
            ("contrast", 0),     # Neutral contrast
            ("aec2", 0),         # Standard AEC
            ("ae_level", 0),     # Neutral auto-exposure
            ("agc", 1),          # auto gain control on
            ("gainceiling", 4),  # allow higher ISO in dark (0-6)
            ("lenc", 1),         # lens correction (fix vignette)
            ("awb", 1),          # auto white balance
            ("awb_gain", 1),     # AWB gain enabled
        ]
        for var, val in params:
            try:
                url = f"{self._base_url}/control?var={var}&val={val}"
                requests.get(url, timeout=2)
            except Exception:
                pass

    def _set_led(self, brightness=64):
        """Set ESP32-CAM flash LED. Lowered to 64/255 to prevent washout."""
        try:
            url = f"{self._base_url}/control?var=led_intensity&val={brightness}"
            requests.get(url, timeout=2)
            print(f"[{self.name}] LED set to {brightness}/255")
        except Exception:
            try:
                requests.get(f"{self._base_url}/led?intensity={brightness}", timeout=2)
            except Exception:
                print(f"[{self.name}] LED control not available")

    def _grab_loop(self, cap):
        """
        High-speed grab thread: drains the OpenCV buffer as fast as possible
        so the publish thread always gets the most recent frame, not a stale one.
        """
        while True:
            ret = cap.grab()  # grab without decode — minimal CPU
            if not ret:
                self._frame_event.set()  # wake publish thread to reconnect
                return
            # Decode only when the publish thread is ready for a new frame
            if not self._frame_event.is_set():
                ret2, frame = cap.retrieve()
                if ret2:
                    with self._frame_lock:
                        self._latest_frame = frame
                    self._frame_event.set()

    def start(self):
        print(f"[{self.name}] Connecting to {self.stream_url}")
        self._set_led(100)
        self._set_camera_params()

        reconnect_delay = 1
        while True:
            cap = cv2.VideoCapture(self.stream_url)

            # Disable OpenCV's internal frame buffer (keep only 1 frame)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not cap.isOpened():
                print(f"[{self.name}] Reconnecting in {reconnect_delay}s...")
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2, self.MAX_RECONNECT_DELAY)
                continue

            reconnect_delay = 1
            self._frame_event.clear()
            self._latest_frame = None

            # Start the dedicated grab thread
            grab_thread = threading.Thread(target=self._grab_loop, args=(cap,), daemon=True)
            grab_thread.start()

            # Publish loop: pushes latest frame downstream at up to 30 FPS
            while grab_thread.is_alive():
                got = self._frame_event.wait(timeout=2.0)
                if not got:
                    break  # timeout → reconnect
                self._frame_event.clear()
                with self._frame_lock:
                    frame = self._latest_frame
                if frame is not None:
                    self.color_image.publish(frame)

            cap.release()
            print(f"[{self.name}] Stream lost — reconnecting...")
