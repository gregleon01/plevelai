# Post-Restructure Checks

Quick steps to confirm the new layout and paths are healthy.

## Offline (no hardware, no camera)
1. Run the smoke test (creates a temp homography + log, dry-run only):
   ```bash
   ./scripts/smoke_offline.sh
   ```
   Expect to see a `[dry-run]` move payload; no serial port needed.

## On Jetson (hardware nearby)
1. Place/point your weights at `models/best.pt` (or set `MODEL=/path/to/your.pt`).
2. Ensure `calibration/H_img_to_ground.npy` exists (or set `homography_path` in `configs/robot.yaml`).
3. Run end-to-end:
   ```bash
   ./launch          # CSI by default; logs to ~/plevelai/detections.log
   ```
4. Watch the runtime console for `[dry-run]` (if DRY_RUN=1) or serial acks (if connected).
5. Optionally open the dashboard:
   ```bash
   ./dashboard
   # open http://<jetson>:8000
   ```

## If something fails
- Import errors: verify `PYTHONPATH` includes `./src` (scripts already set it).
- Missing homography: generate or drop an identity file at `calibration/H_img_to_ground.npy` for quick tests.
- Serial errors: rerun with `DRY_RUN=1` or set `SERIAL_PORT` correctly.
