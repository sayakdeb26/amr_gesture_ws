# ~/amr_gesture_ws/tools/vlm_stub.py
from fastapi import FastAPI, UploadFile, File, Form
from typing import Optional
import time

app = FastAPI()

@app.get("/healthz")
def healthz():
    return {"ok": True}

@app.post("/infer_clip")
async def infer_clip(
    clip: UploadFile = File(...),
    label_hint: Optional[str] = Form(None),
):
    # Simulate fast GPU path and return a reasonable default
    t0 = time.time()
    content = await clip.read()  # consume upload
    latency_ms = int((time.time() - t0) * 1000)
    # Return a predictable label so your UI flow works
    return {
        "label": "WAVE_STOP",
        "confidence": 0.72,
        "rationale": "Stub VLM: waving gesture resembling stop",
        "latency_ms": latency_ms,
        "thumb_url": None
    }
