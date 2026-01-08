# YOLO Live (Jetson, JetPack 6.2)

- CSI via `nvarguscamerasrc` (or USB with `--usb INDEX`)
- Prefers `.engine` if present next to `.pt`
- `--show` opens a window on the Jetson monitor; set `OUT=...` to save MP4 headless.

## Quickstart
```bash
chmod +x run_yolo_live.sh
ln -sf /ssd/yolo/best_1.pt ../../models/best.pt   # or copy your model here
export DISPLAY=:0; xhost +SI:localuser:$USER >/dev/null 2>&1 || true
./run_yolo_live.sh ../../models/best.pt --csi --show
