#!/usr/bin/env python3
"""
One-time homography calibration for the overhead camera.

Grabs a frame, you CLICK 4 reference points on the rink floor in this order:
    1) bottom-left   2) bottom-right   3) top-right   4) top-left
whose real-world (X, Y) meter coordinates you pass via --world. It computes the
pixel->world homography and saves it to homography.npy (used by overhead_tracker.py).

Tip: mark the 4 points with tape on the floor and measure their XY in your rink
frame (e.g. origin at one corner, X across, Y up the rink).

Usage:
    python3 calibrate_homography.py --camera 0 \
        --world 0,0 3,0 3,2 0,2 --out homography.npy
"""

import argparse
import numpy as np
import cv2


def parse_world(pairs):
    pts = [tuple(float(v) for v in p.split(',')) for p in pairs]
    assert len(pts) == 4, 'Need exactly 4 world points'
    return np.array(pts, dtype=np.float32)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='0', help='index or RTSP/URL')
    ap.add_argument('--world', nargs=4, required=True,
                    help='4 world XY in meters: "x,y" BL BR TR TL')
    ap.add_argument('--out', default='homography.npy')
    args = ap.parse_args()

    world = parse_world(args.world)
    src = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(src)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError('Could not read a frame from the camera.')

    clicks = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))
            cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(frame, str(len(clicks)), (x + 8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.namedWindow('calibrate')
    cv2.setMouseCallback('calibrate', on_mouse)
    print('Click 4 points: BL, BR, TR, TL. Press q when done.')
    while True:
        cv2.imshow('calibrate', frame)
        if cv2.waitKey(20) & 0xFF == ord('q') or len(clicks) == 4:
            break
    cv2.destroyAllWindows()

    if len(clicks) != 4:
        raise RuntimeError('Need 4 clicks.')
    pix = np.array(clicks, dtype=np.float32)
    H = cv2.getPerspectiveTransform(pix, world)   # pixel -> world
    np.save(args.out, H)
    print(f'Saved homography to {args.out}\n{H}')
    # quick sanity check
    for uv, w in zip(pix, world):
        p = H @ np.array([uv[0], uv[1], 1.0])
        print(f'  pixel {uv} -> {p[:2] / p[2]}  (expected {w})')


if __name__ == '__main__':
    main()
