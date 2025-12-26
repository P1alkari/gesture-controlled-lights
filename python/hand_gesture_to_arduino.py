import cv2
import mediapipe as mp
import serial
import time
from collections import deque

# === KONFIG ===
PORT = "COM5"   # BYTT til riktig port
BAUD = 9600

HISTORY_LEN = 7  # smoothing-lengde

# === SERIELL ===
use_serial = True
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print("Seriellport OK:", PORT)
except Exception as e:
    print("Klarte ikke å åpne seriellport:", e)
    print("Kjører videre UTEN Arduino (simulering).")
    ser = None
    use_serial = False


def send_count(n: int):
    """Send antall fingre (0–5) til Arduino."""
    n = max(0, min(5, n))
    c = str(n)
    if use_serial:
        ser.write(c.encode())
    print("Fingers →", n)


# === MEDIAPIPE ===
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Får ikke åpnet kameraet")
print("Kamera OK, starter hånd + LED-kontroll... (ESC for å avslutte)")

last_sent = None

# Historikk per finger: [tommelen, index, lang, ring, lille]
finger_histories = [deque(maxlen=HISTORY_LEN) for _ in range(5)]


while True:
    ret, frame = cap.read()
    if not ret:
        print("Klarte ikke å lese fra kameraet")
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    current_extended = [False, False, False, False, False]  # T, I, M, R, L

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        lm = hand_landmarks.landmark

        # Håndstørrelse som skala – avstand i y mellom håndledd og midtfinger-MCP
        wrist = lm[0]
        middle_mcp = lm[9]
        hand_size_y = abs(middle_mcp.y - wrist.y)
        hand_size_x = hand_size_y
        if hand_size_y < 1e-6:
            hand_size_y = 1e-6

        # Terskler (kan tunes)
        FINGER_MARGIN = 0.25 * hand_size_y
        THUMB_MARGIN  = 0.18 * hand_size_x  # litt lavere enn før for å fange tommelen bedre

        # ===== TOMMEL (index 0) =====
        thumb_tip = lm[4]
        thumb_ip  = lm[3]

        dx = thumb_tip.x - thumb_ip.x
        # Tommelen er "ute" hvis den er tydelig til siden av leddet under
        thumb_extended = abs(dx) > THUMB_MARGIN
        current_extended[0] = thumb_extended

        # ===== ANDRE 4 FINGRE =====
        tip_ids = [8, 12, 16, 20]   # fingertupp: index, lang, ring, lille
        pip_ids = [6, 10, 14, 18]   # PIP-ledd (ledd under fingertupp)

        for finger_idx, (tip_id, pip_id) in enumerate(zip(tip_ids, pip_ids), start=1):
            tip = lm[tip_id]
            pip = lm[pip_id]
            # tip må være tydelig høyere enn pip (i bildet: mindre y)
            extended = (pip.y - tip.y) > FINGER_MARGIN
            current_extended[finger_idx] = extended

        mp_drawing.draw_landmarks(
            frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
        )
    else:
        current_extended = [False, False, False, False, False]

    # Oppdater historikk per finger
    for i in range(5):
        finger_histories[i].append(1 if current_extended[i] else 0)

    # Smoothing: finger oppe hvis den er oppe i > 60 % av history
    smoothed_extended = []
    for hist in finger_histories:
        if len(hist) == 0:
            smoothed_extended.append(False)
        else:
            avg = sum(hist) / len(hist)
            smoothed_extended.append(avg > 0.6)

    finger_count_raw = sum(current_extended)
    finger_count_smooth = sum(smoothed_extended)

    if str(finger_count_smooth) != last_sent:
        send_count(finger_count_smooth)
        last_sent = str(finger_count_smooth)

    # Debug-tekst
    cv2.putText(frame, f"Raw: {finger_count_raw}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, f"Smooth: {finger_count_smooth}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    label = "".join(
        ["T" if smoothed_extended[0] else "-",
         "I" if smoothed_extended[1] else "-",
         "M" if smoothed_extended[2] else "-",
         "R" if smoothed_extended[3] else "-",
         "L" if smoothed_extended[4] else "-"]
    )
    cv2.putText(frame, f"Fingers: {label}", (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Hand → 5 LEDs (robust)", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

cap.release()
hands.close()
cv2.destroyAllWindows()
if use_serial:
    ser.close()
