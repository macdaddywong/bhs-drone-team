from __future__ import annotations

import argparse

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Basic webcam face detection with OpenCV")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument(
        "--scale-factor",
        type=float,
        default=1.1,
        help="How much the image size is reduced at each scale (default: 1.1)",
    )
    parser.add_argument(
        "--min-neighbors",
        type=int,
        default=5,
        help="How many neighbors each candidate rectangle should have (default: 5)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        raise RuntimeError(f"Failed to load Haar cascade from: {cascade_path}")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    print("Press 'q' to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Warning: failed to read frame from camera.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=args.scale_factor,
                minNeighbors=args.min_neighbors,
                minSize=(30, 30),
            )

            for x, y, w, h in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (50, 220, 50), 2)

            cv2.putText(
                frame,
                f"Faces: {len(faces)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (30, 200, 255),
                2,
            )

            cv2.imshow("OpenCV Face Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()