#!/usr/bin/env python3
import re
import time

try:
  import cv2
except Exception:
  cv2 = None

from msgq.visionipc import VisionIpcClient, VisionStreamType

from openpilot.common.params import Params


VISION_LIMIT_PARAM = "VisionSpeedLimit"


class VisionSpeedLimitDetector:
  def __init__(self):
    self.params_memory = Params('/dev/shm/params')
    self.params_persistent = Params()

    self.ocr_available = False
    self.ocr_reader = None

    if cv2 is None:
      print("vision_speed_limit: OpenCV unavailable")

    # EasyOCR is lightweight to integrate and works with US/CA signs.
    # Keep it optional to avoid hard failures when dependency is missing.
    try:
      import easyocr
      self.ocr_reader = easyocr.Reader(['en'], gpu=False, verbose=False)
      self.ocr_available = True
    except Exception as error:
      print(f"vision_speed_limit: OCR unavailable ({error})")

    self.regex_limit_text = re.compile(r"\b(SPEED\s*LIMIT|MAXIMUM)\b", re.IGNORECASE)
    self.regex_numbers = re.compile(r"\b(\d{1,3})\b")

  @property
  def enabled(self):
    return self.params_persistent.get_bool("SLCVision") and self.params_persistent.get_bool("SpeedLimitController")

  def connect_camera(self):
    client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    while not client.connect(False):
      if not self.enabled:
        return None
      time.sleep(0.5)
    return client

  @staticmethod
  def crop_sign_region(gray):
    height, width = gray.shape
    # Typical NA speed signs appear right side of lane and upper-middle in perspective.
    y1 = int(height * 0.18)
    y2 = int(height * 0.75)
    x1 = int(width * 0.45)
    x2 = int(width * 0.98)
    return gray[y1:y2, x1:x2]

  def detect_speed_limit_mph(self, frame):
    if cv2 is None or not self.ocr_available:
      return 0

    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi = self.crop_sign_region(gray)

    # Improve OCR on reflective sign text.
    roi = cv2.equalizeHist(roi)
    roi = cv2.GaussianBlur(roi, (3, 3), 0)
    _, thresh = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # easyocr expects RGB image
    rgb = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)

    try:
      detections = self.ocr_reader.readtext(rgb, detail=1, paragraph=False, allowlist='SPEEDLIMITMAXIMUM0123456789')
    except Exception as error:
      print(f"vision_speed_limit: OCR failed ({error})")
      return 0

    for _, text, confidence in detections:
      if confidence < 0.35:
        continue

      upper_text = text.upper().replace("\n", " ")
      if not self.regex_limit_text.search(upper_text):
        continue

      numbers = [int(value) for value in self.regex_numbers.findall(upper_text)]
      for mph in numbers:
        # Reasonable NA posted speed range.
        if 5 <= mph <= 85:
          return float(mph)

    # Fallback: if OCR saw isolated plausible sign number, accept conservative confidence
    for _, text, confidence in detections:
      if confidence < 0.70:
        continue
      numbers = [int(value) for value in self.regex_numbers.findall(text)]
      for mph in numbers:
        if 15 <= mph <= 85:
          return float(mph)

    return 0

  def run(self):
    self.params_memory.put(VISION_LIMIT_PARAM, "0")

    while True:
      if not self.enabled:
        self.params_memory.put(VISION_LIMIT_PARAM, "0")
        time.sleep(1.0)
        continue

      client = self.connect_camera()
      if client is None:
        continue

      while self.enabled:
        buf = client.recv()
        if buf is None:
          time.sleep(0.05)
          continue

        # Road stream is Y plane followed by UV; grayscale from Y plane is sufficient.
        y_plane = buf.data[:buf.width * buf.height]
        frame_gray = y_plane.reshape(buf.height, buf.stride)[:, :buf.width]

        detected_mph = self.detect_speed_limit_mph(frame_gray)
        detected_ms = detected_mph * 0.44704 if detected_mph > 0 else 0
        self.params_memory.put(VISION_LIMIT_PARAM, str(detected_ms))


if __name__ == "__main__":
  VisionSpeedLimitDetector().run()
