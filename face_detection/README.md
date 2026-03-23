uv# Face Detection (OpenCV)

This project runs basic real-time face detection from your webcam using OpenCV and a built-in Haar cascade classifier.

## Requirements

- Python 3.10+
- Webcam access

## Install

If you use `uv`:

```bash
uv sync
```

Or with pip:

```bash
python -m pip install -e .
```

## Run

```bash
python main.py
```

Optional flags:

```bash
python main.py --camera 0 --scale-factor 1.1 --min-neighbors 5
```

Controls:

- Press `q` or `Esc` to quit.
