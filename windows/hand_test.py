import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

options = vision.HandLandmarkerOptions(
    base_options=mp_python.BaseOptions(model_asset_path="hand_landmarker.task"),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=1,
)
landmarker = vision.HandLandmarker.create_from_options(options)

cap = cv2.VideoCapture(0)
start = time.time()

while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    timestamp_ms = int((time.time() - start) * 1000)
    result = landmarker.detect_for_video(mp_image, timestamp_ms)

    if result.hand_landmarks:
        hand = result.hand_landmarks[0]
        h, w, _ = frame.shape
        for p in hand:
            cv2.circle(frame, (int(p.x * w), int(p.y * h)), 4, (0, 255, 0), -1)
        wrist = hand[0]
        thumb = hand[4]
        index = hand[8]
        pinch = ((thumb.x - index.x) ** 2 + (thumb.y - index.y) ** 2) ** 0.5
        text = f"x={wrist.x:.2f} y={wrist.y:.2f} pinch={pinch:.2f}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
landmarker.close()