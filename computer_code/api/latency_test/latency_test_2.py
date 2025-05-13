import cv2
import numpy as np
import matplotlib.pyplot as plt


class PSEYE:
    def __init__(self):
        from pseyepy import Camera
        self.CAM_INDEX = 0
        self.cams = Camera(fps=240, resolution=Camera.RES_SMALL)
        # Discard the first frame
        self.cams.read()

    def stop(self):
        self.cams.end()

    def get_frame(self):
        frame, timestamp = self.cams.read(self.CAM_INDEX )
        return frame

class GenericCamera:
    def __init__(self):
        CAM_INDEX = 0
        # cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        # cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
        if self.cap is None or not self.cap.isOpened():
            print("Error: Cannot open camera.")
            exit(0)

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        # Set resolution and frame rate
        # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        # Confirm what was accepted
        print("FOURCC:", self.cap.get(cv2.CAP_PROP_FOURCC))
        print("Width:", self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Height:", self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("FPS:", self.cap.get(cv2.CAP_PROP_FPS))

    def stop(self):
        self.cap.release()

    def get_frame(self):
        _, frame = self.cap.read()
        return frame


camera = GenericCamera()
# camera = PSEYE()

frame = camera.get_frame()
frame_height, frame_width = frame.shape[:2]
screen_shape = (frame_height, frame_width, 3)

cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

def calibrate_screen(color_value, label):
    screen_img = np.full(screen_shape, color_value, dtype=np.uint8)

    while True:
        frame = camera.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.average(gray)

        display = screen_img.copy()
        text = f"{label} LEVEL: {avg_brightness:.1f} - Press 's' to set"
        cv2.putText(display, text, (10, frame_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (127, 127, 127), 1)

        cv2.imshow("frame", display)

        if cv2.waitKey(1) & 0xFF == ord('s'):
            return avg_brightness

# Calibration
white_level = calibrate_screen(255, "WHITE")
black_level = calibrate_screen(0, "BLACK")

# Reset screen to white after calibration
cv2.imshow("frame", np.full((frame_height, frame_width), 255, dtype=np.uint8))
cv2.waitKey(1)


THRESHOLD = (white_level + black_level) / 2
print(f"Calculated threshold: {THRESHOLD:.2f}")

prev_tick = cv2.getTickCount()
frame_number, prev_change_frame = 0, 0
is_dark = True
latencies = []

while True:
    frame_number += 1

    frame = camera.get_frame()
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    avg = np.average(img)
    is_now_dark = avg < THRESHOLD

    if is_dark != is_now_dark:
        is_dark = is_now_dark
        new = cv2.getTickCount()

        latency = (new - prev_tick) / cv2.getTickFrequency()
        latencies.append(latency)

        print("{:.3f} sec, {:.3f} frames".format(
            latency,
            frame_number - prev_change_frame
        ))

        prev_tick = new
        prev_change_frame = frame_number

        fill_color = 255 if is_dark else 0
        show = np.full((frame_height, frame_width), fill_color, dtype=img.dtype)

        cv2.imshow('frame', show)

        if len(latencies) >= 100:
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


cv2.destroyAllWindows()

# Final average calculation and statistics
if latencies:
    import numpy as np
    import matplotlib.pyplot as plt

    latencies = np.array(latencies[3:])  # Skip first few unstable values

    latency_mean = np.mean(latencies)
    latency_median = np.median(latencies)
    latency_std = np.std(latencies)
    latency_min = np.min(latencies)
    latency_max = np.max(latencies)

    print(f"\nLatency Statistics (based on {len(latencies)} samples):")
    print(f"  Mean              : {latency_mean:.6f} sec")
    print(f"  Median            : {latency_median:.6f} sec")
    print(f"  Standard Deviation: {latency_std:.6f} sec")
    print(f"  Min               : {latency_min:.6f} sec")
    print(f"  Max               : {latency_max:.6f} sec")

    # Line + histogram plot
    plt.figure(figsize=(12, 6))

    # Line plot
    plt.subplot(1, 2, 1)
    plt.plot(latencies, label="Latency", linestyle='--', marker='o', alpha=0.6)
    plt.axhline(latency_mean, color='r', linestyle=':', label='Mean')
    plt.axhline(latency_median, color='g', linestyle=':', label='Median')
    plt.xlabel("Transition Index")
    plt.ylabel("Latency (seconds)")
    plt.title("Latency Over Time")
    plt.legend()
    plt.grid(True)

    # Histogram
    plt.subplot(1, 2, 2)
    plt.hist(latencies, bins=20, color='gray', edgecolor='black', alpha=0.7)
    plt.axvline(latency_mean, color='r', linestyle='--', label='Mean')
    plt.axvline(latency_median, color='g', linestyle='--', label='Median')
    plt.xlabel("Latency (seconds)")
    plt.ylabel("Frequency")
    plt.title("Latency Distribution")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()