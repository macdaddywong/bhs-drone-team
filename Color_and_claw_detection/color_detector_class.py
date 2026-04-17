import cv2
import numpy as np
import argparse
import sys


class ColorDetector:
    # ──────────────────────────────────────────────
    # HSV Color Ranges
    # ──────────────────────────────────────────────
    COLOR_RANGES = {
        "red": [
            (np.array([0, 120, 70]), np.array([10, 255, 255])),
            (np.array([170, 120, 70]), np.array([180, 255, 255])),
        ],
        "orange": [(np.array([10, 120, 70]), np.array([25, 255, 255]))],
        "yellow": [(np.array([25, 100, 70]), np.array([35, 255, 255]))],
        "green": [(np.array([36, 100, 70]), np.array([86, 255, 255]))],
        "blue": [(np.array([94, 100, 70]), np.array([126, 255, 255]))],
        "purple": [(np.array([127, 50, 70]), np.array([155, 255, 255]))],
        "white": [(np.array([0, 0, 200]), np.array([180, 30, 255]))],
        "black": [(np.array([0, 0, 0]), np.array([180, 255, 50]))],
    }

    DISPLAY_COLORS = {
        "red": (0, 0, 220),
        "orange": (0, 140, 255),
        "yellow": (0, 220, 220),
        "green": (0, 200, 60),
        "blue": (220, 80, 0),
        "purple": (180, 0, 180),
        "white": (230, 230, 230),
        "black": (60, 60, 60),
    }

    MIN_CONTOUR_AREA = 1500
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480

    # ──────────────────────────────────────────────
    # Init
    # ──────────────────────────────────────────────
    def __init__(self, use_picam=False, initial_color=None, index:int=1):
        self.use_picam = use_picam
        #self.detection_logic = detectionLogic()

        self.index = index
        self.all_colors = list(self.COLOR_RANGES.keys())
        self.active_colors = [initial_color] if initial_color else list(self.all_colors)

        self.source = self.open_camera(self.index)

    # ──────────────────────────────────────────────
    # Camera
    # ──────────────────────────────────────────────
    def open_camera(self, index:int=1):
        if self.use_picam:
            return self.open_pi_camera()
        return self.open_usb_camera(index)

    def open_usb_camera(self, index: int = 1):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            sys.exit(f"[ERROR] Could not open USB camera at index {index}.")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 30)

        print(f"[INFO] USB camera opened (index={index}).")
        return cap

    def open_pi_camera(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            sys.exit("[ERROR] picamera2 not found.")

        picam = Picamera2()
        config = picam.create_preview_configuration(
            main={"size": (self.FRAME_WIDTH, self.FRAME_HEIGHT), "format": "BGR888"}
        )
        picam.configure(config)
        picam.start()

        print("[INFO] Pi Camera opened via picamera2.")
        return picam

    def read_frame(self):
        if self.use_picam:
            return self.source.capture_array()

        ret, frame = self.source.read()
        if not ret:
            sys.exit("[ERROR] Failed to grab frame.")
        return frame

    # ──────────────────────────────────────────────
    # Detection
    # ──────────────────────────────────────────────
    def build_mask(self, hsv, color_name):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lo, hi in self.COLOR_RANGES[color_name]:
            mask |= cv2.inRange(hsv, lo, hi)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        return mask

    def detect_and_annotate(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        output = frame.copy()
        detected_colors_found = []

        for color_name in self.active_colors:
            mask = self.build_mask(hsv, color_name)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            bgr = self.DISPLAY_COLORS.get(color_name, (255, 255, 255))
            found_this_color = False

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.MIN_CONTOUR_AREA:
                    continue

                found_this_color = True

            if found_this_color:
                detected_colors_found.append(color_name)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.MIN_CONTOUR_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2

                cv2.rectangle(output, (x, y), (x + w, y + h), bgr, 2)

                label = f"{color_name} {int(area)}px"
                (tw, th), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
                )

                cv2.rectangle(output, (x, y - th - baseline - 4), (x + tw + 4, y), bgr, -1)

                txt_color = (0, 0, 0) if color_name not in ("black", "blue", "purple") else (255, 255, 255)
                cv2.putText(output, label, (x + 2, y - baseline - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, txt_color, 1, cv2.LINE_AA)

                cv2.circle(output, (cx, cy), 5, bgr, -1)
        
        print(f"DETECTED COLORS FOUND: {detected_colors_found}\n")
        return output, detected_colors_found

    # ──────────────────────────────────────────────
    # HUD
    # ──────────────────────────────────────────────
    def draw_hud(self, frame):
        overlay = frame.copy()
        h, w = frame.shape[:2]

        cv2.rectangle(overlay, (0, h - 34), (w, h), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        instructions = "Keys: [1-8] toggle color [a] all [n] none [q] quit"
        cv2.putText(frame, instructions, (8, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        pill_x = 8
        for i, name in enumerate(self.all_colors):
            active = name in self.active_colors
            bgr = self.DISPLAY_COLORS[name] if active else (70, 70, 70)
            label = f"{i+1}:{name}"

            (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)

            cv2.rectangle(frame, (pill_x, 6), (pill_x + tw + 8, 24),
                          bgr, -1 if active else 1)

            txt_c = (0, 0, 0) if active else (220, 220, 220)
            cv2.putText(frame, label, (pill_x + 4, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, txt_c, 1, cv2.LINE_AA)

            pill_x += tw + 14

        return frame

    # ──────────────────────────────────────────────
    # Controls
    # ──────────────────────────────────────────────
    def handle_key(self, key):
        if key == ord('q'):
            return False

        elif key == ord('a'):
            self.active_colors = list(self.all_colors)

        elif key == ord('n'):
            self.active_colors = []

        elif ord('1') <= key <= ord('8'):
            idx = key - ord('1')
            name = self.all_colors[idx]

            if name in self.active_colors:
                self.active_colors.remove(name)
            else:
                self.active_colors.append(name)

            if "red" in self.active_colors:
                print(f"[INFO] RED detected: {self.active_colors}")
                self.detection_logic.gradually_increase_scale(15)

        return True

    # ──────────────────────────────────────────────
    # Main loop
    # ──────────────────────────────────────────────
    def run(self):
        print("[INFO] Starting detection...")

        while True:
            frame = self.read_frame()
            output, detected_colors = self.detect_and_annotate(frame)
            output = self.draw_hud(output)

            try:
                cv2.imshow("Color Detection", output)
            except:
                print("frame processed")
                cv2.imwrite("debug.jpg", frame)

            key = cv2.waitKey(1) & 0xFF
            if not self.handle_key(key):
                break

        self.cleanup()

    def cleanup(self):
        cv2.destroyAllWindows()
        if self.use_picam:
            self.source.stop()
        else:
            self.source.release()
        print("[INFO] Clean exit.")


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────
if __name__ == "__main__":
    

    parser = argparse.ArgumentParser()
    parser.add_argument("--picam", action="store_true")
    parser.add_argument("--color", type=str, default=None,
                        choices=list(ColorDetector.COLOR_RANGES.keys()))

    args = parser.parse_args()

    detector = ColorDetector(use_picam=args.picam, initial_color=args.color, index=0)
    detector.run()