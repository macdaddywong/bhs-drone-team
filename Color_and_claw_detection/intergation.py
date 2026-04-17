import cv2
import time
import random
import os

from color_detector_class import ColorDetector   # your refactored class file
from chatgpt_detectionclaw import DetectionClaw   # your claw class file
from utils import Utils


class IntegrationTest:
    """
    Connects ColorDetector → DetectionClaw
    """

    def __init__(self, use_picam=False, simulate_object=False, index:int=0):
        self.detector = ColorDetector(use_picam=use_picam, index=index)
        self.claw = DetectionClaw(index=index)

        self.utils = Utils()
        self.debug = self.utils.debug
        self.reset = self.utils.reset
        self.quick_doc = self.utils.quick_doc

        self.simulate_object = simulate_object

        # optional simulated objects
        self.fake_objects = ["turtle", "ball", "cup", "none"]

    # ---------------- MAIN LOOP ----------------

    def run(self):
        self.reset()
        print()
        self.quick_doc("[TEST] Starting integration test...")

        while True:
            frame = self.detector.read_frame()
            self.debug(f"FRAME SHAPE: {frame.shape}")
            output, detected_colors = self.detector.detect_and_annotate(frame)
            self.debug(f"1st OUTPUT SHAPE: {output.shape}")
            output = self.detector.draw_hud(output)
            self.debug(f"2nd OUTPUT SHAPE: {output.shape}")

            # ---- extract "detected colors" ----
            #detected_colors = self.detector.active_colors
            self.debug(f"DETECTED COLORS: {detected_colors}")

            # ---- optional fake object detection ----
            detected_object = self._get_object()
            self.debug(f"DETECTED OBJECT: {detected_object}")

            # ---- send to claw ---- 
            self.quick_doc("[intergation.py at run()] NEW LOOPING IN DETECTED COLORS")

            for color in detected_colors:
               
                self.claw.process_detection(color, detected_object)

            # ---- show video ----
            key = 255

            if "DISPLAY" in os.environ:
                cv2.imshow("Integration Test", output)
                key = cv2.waitKey(1) & 0xFF
            else:
                # headless mode
                cv2.imwrite("debug.jpg", output)

            # optional slow-down for debugging
            time.sleep(0.03)

        self.cleanup()
        self.reset()

    # ---------------- OBJECT SIMULATION ----------------

    def _get_object(self):
        if not self.simulate_object:
            return ""

        return random.choice(self.fake_objects)

    # ---------------- CLEANUP ----------------

    def cleanup(self):
        cv2.destroyAllWindows()

        if self.detector.use_picam:
            self.detector.source.stop()
        else:
            self.detector.source.release()

        print("[TEST] Finished cleanly.")


# ---------------- ENTRY POINT ----------------

if __name__ == "__main__":

    index = 0
    while True:
        try:
            user = int(input("Which camera index do you want?: "))
            if user not in [0,1,2]:
                print("Please insert a valid inetegr (0,1,2)")
                continue
            else:
                index = user
                break

        except:
            print("Please insert a valid inetegr (0,1,2 - depends on how many cameras are in your system)")
            continue

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--picam", action="store_true")
    parser.add_argument("--simulate-object", action="store_true")

    args = parser.parse_args()

    test = IntegrationTest(
        use_picam=args.picam,
        simulate_object=args.simulate_object,
        index=index
    )

    test.run()