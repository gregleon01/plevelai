from __future__ import annotations

import itertools
import time
from pathlib import Path
from typing import Dict

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .service import DetectionService

app = FastAPI(title="Weeder Dashboard", version="0.1.0")
service = DetectionService()


@app.get("/api/status")
def api_status() -> Dict:
    return service.status()


@app.get("/api/events")
def api_events(limit: int = 50) -> Dict:
    limit = max(1, min(limit, 200))
    return {"events": service.events(limit)}


@app.get("/video")
def video_stream() -> StreamingResponse:
    boundary = "frame"

    def frame_generator():
        for _ in itertools.count():
            jpeg = service.latest_jpeg()
            if jpeg is None:
                time.sleep(0.05)
                continue
            yield (
                b"--" + boundary.encode() + b"\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            )

    return StreamingResponse(
        frame_generator(),
        media_type=f"multipart/x-mixed-replace; boundary={boundary}",
    )


ROOT_DIR = Path(__file__).resolve().parents[4]
STATIC_DIR = ROOT_DIR / "web" / "dashboard"
if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=STATIC_DIR, html=True), name="static")


@app.get("/")
def index() -> FileResponse:
    index_path = STATIC_DIR / "index.html"
    if not index_path.exists():
        raise HTTPException(404, detail="Dashboard assets missing (build frontend)")
    return FileResponse(index_path)


@app.on_event("shutdown")
def shutdown_event() -> None:
    service.stop()
