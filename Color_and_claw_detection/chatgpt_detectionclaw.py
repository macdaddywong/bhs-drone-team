import time
import platform
from gpiozero import Servo

class DetectionClaw:

    def __init__(self, focused_color:str="red", focused_object:str="turtle", index:int=0):

        self.focused_color: str = focused_color.lower()
        self.focused_object: str = focused_object.lower()
        self.index: int = index
        self.is_open: bool = False

        # -------- SERVO SETUP --------
        if platform.system() == "Windows":
            self.servo = self._fake_servo()
        else:
            
            self.servo = Servo(17)

        # Tune these values for YOUR claw
        self.OPEN_POS = 0.7
        self.CLOSED_POS = -0.4

        self.close_claw()  # start closed

    # ---------------- DETECTION ----------------

    def process_detection(self, color: str, obj: str = ""):
        color = color.lower()
        obj = obj.lower()

        if self._should_open(color, obj):
            self.open_claw()
        else:
            self.close_claw()

    def _should_open(self, color, obj):
        return ("red" in color) or (self.focused_object in obj)

    # ---------------- SERVO CONTROL ----------------

    def open_claw(self):
        if not self.is_open:
            print("Opening claw")
            self.servo.value = self.OPEN_POS
            self.is_open = True
            time.sleep(0.3)

    def close_claw(self):
        if self.is_open:
            print("Closing claw")
            self.servo.value = self.CLOSED_POS
            self.is_open = False
            time.sleep(0.3)

    # ---------------- MOCK SERVO ----------------

    def _fake_servo(self):
        class Fake:
            def __init__(self):
                self.value = 0

            def __setattr__(self, name, value):
                print(f"[MOCK SERVO] {name} = {value}")

        return Fake()


# ---------------- TEST ----------------

if __name__ == "__main__":
    claw = DetectionClaw()

    while True:
        color = input("Detected color: ")
        obj = input("Detected object: ")

        claw.process_detection(color, obj)