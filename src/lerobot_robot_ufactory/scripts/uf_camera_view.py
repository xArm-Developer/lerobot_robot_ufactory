#!/usr/bin/env python
"""Multi-camera viewer: XVisio / RealSense / other cameras."""
import os
import sys
import cv2
import time
import argparse
import threading
from pathlib import Path

# ---------- 屏蔽 OpenCV Qt 字体警告 ----------
os.environ["QT_LOGGING_RULES"] = "*=false"
_fake_stderr = open(os.devnull, "w")
_real_stderr = sys.stderr
sys.stderr = _fake_stderr
import cv2 as _cv2
sys.stderr = _real_stderr
_fake_stderr.close()
# ---------------------------------------------

BY_ID_DIR = Path("/dev/v4l/by-id")
BY_PATH_DIR = Path("/dev/v4l/by-path")

# camera type -> (keyword, default format)
CAMERA_TYPES = {
    "xvisio":    (1280, 1280, "YU12"),
    "realsense": (640,  480,  ""),
}


class Camera:
    """Generic camera."""

    def __init__(self, device: str, serial: str = "", by_path: str = "",
                 by_id: str = "", width: int = 640, height: int = 480,
                 fourcc: str = ""):
        self.device = device
        self.by_path = by_path
        self.by_id = by_id
        self.serial = serial
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self._cap: cv2.VideoCapture | None = None

    def open(self) -> bool:
        for b in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            cap = cv2.VideoCapture(self.device, b)
            if cap.isOpened():
                if self.fourcc:
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*self.fourcc))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                if rw < 1 or rh < 1:
                    cap.release()
                    continue
                self.width, self.height = rw, rh
                self._cap = cap
                return True
        return False

    def read(self):
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        return frame if ret else None

    def close(self):
        if self._cap:
            self._cap.release()
            self._cap = None

    @property
    def fps(self) -> float:
        return self._cap.get(cv2.CAP_PROP_FPS) if self._cap else 0.0

    @property
    def current_fourcc(self) -> str:
        if not self._cap:
            return ""
        code = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        try:
            return "".join(chr((code >> (i * 8)) & 0xFF) for i in range(4))
        except (ValueError, OverflowError):
            return str(code)

    @property
    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()


class CameraManager:
    """Camera discovery."""

    @staticmethod
    def find(cam_type: str = "other") -> list[dict]:
        """Scan cameras by type.
        cam_type: xvisio / realsense / other (default, excludes known types)
        Uses /dev/v4l/by-path/ (unique per port), gets serial from by-id.
        video-index0 = Video Capture, index1+ = depth/IR/metadata.
        """
        if not BY_PATH_DIR.exists():
            return []

        info = CAMERA_TYPES.get(cam_type)
        if info:
            keyword = cam_type
            w, h, fc = info
        else:
            keyword = None
            w, h, fc = (640, 480, "")

        # 建立 by-id 映射: resolved device -> by-id path
        id_map = {} # {'/dev/video0': '/dev/v4l/by-id/xxxx'}
        if BY_ID_DIR.exists():
            for p in BY_ID_DIR.iterdir():
                id_map[str(p.resolve())] = str(p)

        result = []
        for p in sorted(BY_PATH_DIR.iterdir()):
            name = p.name
            if not name.endswith("-video-index0"):
                continue

            device = str(p.resolve())
            by_id = id_map.get(device, "")

            # use by-id name for type check (by-path does not contain vendor/model)
            type_name = (by_id or name).lower()
            if keyword and keyword not in type_name:
                continue
            if cam_type == 'other' and not keyword and any(k in type_name for k in CAMERA_TYPES.keys()):
                continue

            if by_id:
                # serial = by_id.rsplit("-video-index0", 1)[0]
                # parts = serial.split("_", 2)
                # serial = parts[-1] if len(parts) > 2 else serial
                serial = by_id.rsplit('/', 1)[-1].split('-', 1)[-1].rsplit("-video-index0", 1)[0]
                tmp = serial.split("_")
                serial = '_'.join(list({val: val for val in tmp if val}.values()))
            else:
                serial = name.rsplit("-video-index0", 1)[0]

            result.append({
                "device": device,
                "serial": serial,
                "by_path": str(p),
                "by_id": by_id,
                "default_w": w,
                "default_h": h,
                "default_fourcc": fc,
            })
        return result

    @staticmethod
    def find_by_serial(serial: str, cam_type: str = "other") -> list[dict]:
        all_devs = CameraManager.find(cam_type)
        return [d for d in all_devs if serial in d["by_id"] or serial in d["by_path"] or serial in d["serial"]]
        # return [d for d in all_devs if serial in d["serial"]]


class MultiCameraViewer:
    """Multi-camera stitched viewer."""

    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 1.2
    FONT_THICK = 2
    LABEL_H = 66

    def __init__(self, cameras: list[Camera], seconds: int = 0):
        self.cameras = cameras
        self.seconds = seconds
        self._max_w = int(os.environ.get("DISPLAY_W", 3840))
        self._max_h = int(os.environ.get("DISPLAY_H", 2160))

    def _stitch_frames(self, frames: list, start_monotonic: float = 0):
        valid = [(cam, f) for cam, f in zip(self.cameras, frames) if f is not None]
        if not valid:
            return None

        resized = []
        seconds = int(time.monotonic() - start_monotonic)
        for cam, frame in valid:
            h, w = frame.shape[:2]
            max_per_cam = self._max_h * 0.8
            if h > max_per_cam:
                scale = max_per_cam / h
                frame = cv2.resize(frame, (int(w * scale), int(h * scale)))

            label1 = cam.serial
            label2 = f"{w}x{h} | {cam.fps:.0f}fps | {cam.current_fourcc} | #{seconds}"
            padded = cv2.copyMakeBorder(frame, self.LABEL_H, 0, 0, 0,
                                        cv2.BORDER_CONSTANT, value=(40, 40, 40))
            cv2.putText(padded, label1, (10, 30),
                        self.FONT, self.FONT_SCALE, (0, 255, 255), self.FONT_THICK, cv2.LINE_AA)
            cv2.putText(padded, label2, (10, 80),
                        self.FONT, self.FONT_SCALE, (0, 200, 255), self.FONT_THICK, cv2.LINE_AA)
            resized.append(padded)

        max_h = max(f.shape[0] for f in resized)
        aligned = []
        for i, f in enumerate(resized):
            if f.shape[0] != max_h:
                f = cv2.resize(f, (int(f.shape[1] * max_h / f.shape[0]), max_h))
            # 加右边分隔线（100, 100, 100 灰色，4px 宽）
            f = cv2.copyMakeBorder(f, 0, 0, 0, 4,
                                   cv2.BORDER_CONSTANT, value=(100, 100, 100))
            aligned.append(f)

        result = cv2.hconcat(aligned)
        rh, rw = result.shape[:2]
        scale = min(self._max_w / rw, self._max_h / rh, 1.0)
        if scale < 1.0:
            result = cv2.resize(result, (int(rw * scale), int(rh * scale)))
        return result

    def run(self):
        has_gui = bool(os.environ.get("DISPLAY", ""))
        if not has_gui:
            print("No GUI, press Ctrl+C to exit.")

        if has_gui:
            cv2.namedWindow("Camera Viewer", cv2.WINDOW_NORMAL)

        # 用 pynput 全局监听键盘（不依赖 OpenCV 窗口焦点）
        stop_event = threading.Event()
        if has_gui:
            from pynput import keyboard
            def _on_press(key):
                try:
                    if key == keyboard.Key.esc or (hasattr(key, 'char') and key.char == 'q'):
                        stop_event.set()
                except Exception:
                    pass
            kb_listener = keyboard.Listener(on_press=_on_press)
            kb_listener.daemon = True
            kb_listener.start()

        print(f"\nConnected {len(self.cameras)} cameras:\n")
        for i, cam in enumerate(self.cameras):
            print(f"[{i+1}] {cam.serial} ({cam.width}x{cam.height} {cam.fourcc or 'default'})")
            print(f"    video_path: {cam.device}")
            print(f"    v4l_path:   {cam.by_path}")
            if cam.by_id:
                print(f"    v4l_id:     {cam.by_id}")
            print()
        print("\nPress q or Esc to exit.\n")

        expired = 0 if self.seconds <= 0 else time.monotonic() + self.seconds
        frame_count = 0
        start_monotonic = time.monotonic()

        try:
            while not stop_event.is_set() and (expired == 0 or time.monotonic() < expired):
                frames = [cam.read() for cam in self.cameras]
                frame_count += 1

                if has_gui:
                    stitched = self._stitch_frames(frames, start_monotonic)
                    if stitched is not None:
                        if frame_count == 1:
                            cv2.resizeWindow("Camera Viewer",
                                             stitched.shape[1], stitched.shape[0])
                        cv2.imshow("Camera Viewer", stitched)
                    cv2.waitKey(1)
                    # exit if window was closed
                    try:
                        if cv2.getWindowProperty("Camera Viewer", cv2.WND_PROP_VISIBLE) < 1:
                            break
                    except cv2.error:
                        break
                else:
                    if frame_count % 30 == 0:
                        parts = []
                        for cam, f in zip(self.cameras, frames):
                            status = "OK" if f is not None else "FAIL"
                            parts.append(f"{cam.device}:{status}")
                        print(f"[{time.strftime('%H:%M:%S')}] #{frame_count} | "
                              + " | ".join(parts), end="\r")
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            if has_gui:
                cv2.destroyAllWindows()
            for cam in self.cameras:
                cam.close()
            print("\nExited.")


def main():
    parser = argparse.ArgumentParser(
        description="Multi-camera stitched viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  uf-camera-view -l                        # list all cameras
  uf-camera-view -T xvisio -l             # list all XVisio vSLAM
  uf-camera-view -T realsense -l          # list all RealSense
  uf-camera-view -T xvisio                # show all XVisio (default 1280x1280 YU12)
  uf-camera-view -T xvisio -s 250801DR48  # filter by serial number
  uf-camera-view -T xvisio -W 1280 -H 1280 -F YU12  # specify format (main camera)
  uf-camera-view -T xvisio -W 640 -H 1920 -F NV12  # specify format (aux cameras)
  uf-camera-view -T other                # show other cameras
  uf-camera-view -T other -t 30           # preview 30 seconds
  DISPLAY_W=2560 DISPLAY_H=1440 uf-camera-view -T xvisio  # set window size limit""")
    parser.add_argument("-T", "--type", type=str, default=None,
                        choices=["xvisio", "realsense", "other", "all"],
                        help="camera type")
    parser.add_argument("-s", "--serial", type=str, default=None,
                        help="filter by serial number")
    parser.add_argument("-l", "--list", action="store_true",
                        help="list devices of the given type")
    parser.add_argument("-t", "--time", type=int, default=0,
                        help="preview seconds, 0 = infinite (default: 0)")
    parser.add_argument("-W", "--width", type=int, default=None,
                        help="capture width")
    parser.add_argument("-H", "--height", type=int, default=None,
                        help="capture height")
    parser.add_argument("-F", "--fourcc", type=str, default=None,
                        help="pixel format (e.g. YU12, NV12, MJPG)")
    if len(sys.argv) == 1:
        parser.print_help()
        return

    args = parser.parse_args()

    # -l without -T defaults to all; no -l and no -T is an error
    if args.list and not args.type:
        args.type = "all"
    elif not args.list and not args.type:
        parser.error("-T/--type is required (xvisio, realsense, other, all)")

    devices = CameraManager.find(args.type)

    if args.list:
        if devices:
            type_label = {"all": "", "other": "Other "}.get(args.type, f'{args.type} ')
            print(f"Found {len(devices)} {type_label}camera(s):\n")
            for i, d in enumerate(devices):
                print(f"[{i+1}] {d['serial']}")
                print(f"    video_path: {d['device']}")
                print(f"    v4l_path:   {d['by_path']}")
                if d.get("by_id"):
                    print(f"    v4l_id:     {d['by_id']}")
                print()
        else:
            print(f"No {args.type} cameras found")
        return

    if args.serial:
        devices = CameraManager.find_by_serial(args.serial, args.type)

    if not devices:
        print(f"No {args.type} cameras found. Use -l to list available devices.")
        return

    cameras = []
    for d in devices:
        w = args.width if args.width else d["default_w"]
        h = args.height if args.height else d["default_h"]
        fc = args.fourcc if args.fourcc else d["default_fourcc"]
        cam = Camera(d["device"], serial=d["serial"],
                     by_path=d["by_path"], by_id=d.get("by_id", ""),
                     width=w, height=h, fourcc=fc)
        if cam.open():
            cameras.append(cam)
        else:
            print(f"WARNING: cannot open {d['device']}")

    if not cameras:
        print("No available cameras.")
        return

    viewer = MultiCameraViewer(cameras, seconds=args.time)
    viewer.run()


if __name__ == "__main__":
    main()