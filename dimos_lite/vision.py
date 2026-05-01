import cv2
import requests
import time
import threading
import numpy as np
from ultralytics import YOLO
import torch

from dimos_lite.core import Module, StreamOut

class VisionModule(Module):
    MAX_RECONNECT_DELAY = 10

    def __init__(self, stream_url="http://192.168.4.1:81/stream"):
        super().__init__("VisionV4")
        self.stream_url = stream_url
        self.color_image = StreamOut("color_image")
        self.detections = StreamOut("detections") 
        self._model = YOLO('yolov8n.pt')
        self._device = "mps" if torch.backends.mps.is_available() else "cpu"
        self._base_url = stream_url.rsplit('/', 1)[0].replace(':81', '')
        self.frame_count = 0
        self._interrupt_stream = False 

    def start(self):
        print(f"[{self.name}] Initializing {self.stream_url}")
        self._set_led(50)
        self._set_camera_params()

        # Start the dedicated stream thread
        stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        stream_thread.start()

    def _set_led(self, brightness=50):
        """Set ESP32-CAM flash LED by temporarily cutting the stream."""
        def _task():
            # 1. Trigger the cut
            self._interrupt_stream = True
            time.sleep(0.15) # Quick wait for stream to close
            
            state_str = "on" if brightness > 0 else "off"
            headers = {"Connection": "close"}
            url = f"{self._base_url}/led?state={state_str}"
            try:
                requests.get(url, headers=headers, timeout=1.0)
            except Exception:
                pass
            finally:
                self._interrupt_stream = False
        
        threading.Thread(target=_task, daemon=True).start()
        return True

    def _set_camera_params(self):
        """Set resolution and flip for ESP32-CAM."""
        params = [("framesize", 6), ("quality", 10)]
        for var, val in params:
            try:
                url = f"{self._base_url}/control?var={var}&val={val}"
                requests.get(url, timeout=2)
            except Exception:
                pass

    def _stream_loop(self):
        """
        Robust M-JPEG stream reader with 'Silence & Strike' support.
        """
        while True:
            try:
                print(f"[{self.name}] Opening HTTP stream...")
                resp = requests.get(self.stream_url, stream=True, timeout=5)
                if resp.status_code != 200:
                    time.sleep(2)
                    continue

                byte_buffer = b""
                for chunk in resp.iter_content(chunk_size=512):
                    if self._interrupt_stream:
                        print(f"[{self.name}] INTERRUPTING stream for LED command...")
                        resp.close()
                        break
                        
                    byte_buffer += chunk
                    a = byte_buffer.find(b'\xff\xd8') # JPEG Start
                    b = byte_buffer.find(b'\xff\xd9') # JPEG End

                    if a != -1 and b != -1:
                        jpg = byte_buffer[a:b+2]
                        byte_buffer = byte_buffer[b+2:]
                        buf = np.frombuffer(jpg, dtype=np.uint8)

                        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                        if frame is not None and frame.size > 0:
                            frame = cv2.flip(frame, 0) # Vertical flip only (removes mirroring)
                            
                            # YOLOv8 Detection
                            results = self._model(frame, stream=False, verbose=False, device=self._device)
                            current_detections = []
                            for r in results:
                                for box in r.boxes:
                                    cls = int(box.cls[0])
                                    conf = float(box.conf[0])
                                    xyxy = box.xyxy[0].tolist()
                                    name = self._model.names[cls]
                                    current_detections.append({
                                        "name": name,
                                        "label": name, # Support both 'name' and 'label' for agent compatibility
                                        "conf": conf,
                                        "confidence": conf,
                                        "bbox": xyxy
                                    })

                            self.detections.publish(current_detections)
                            self.color_image.publish(frame)
                            self.frame_count += 1
                
                if self._interrupt_stream:
                    time.sleep(0.1) # Minimum gap
                    
            except Exception as e:
                if not self._interrupt_stream:
                    print(f"[{self.name}] Stream error: {e}")
                time.sleep(1)
