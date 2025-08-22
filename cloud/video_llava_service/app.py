# cloud/video_llava_service/app.py
from fastapi import FastAPI, UploadFile, File, Body
from typing import List, Dict, Any
from pydantic import BaseModel
from tempfile import NamedTemporaryFile
from PIL import Image
import io
import os

app = FastAPI()

# ---- existing model / inference stub you already have ----
# Assume you had something like this:
# class InferRequest(BaseModel):
#     frames: List[str]
#     context: Dict[str, Any] = {}
#
# def run_inference_on_paths(frames: List[str], context: Dict[str, Any]) -> Dict[str, Any]:
#     # your existing logic; returns {"label": "...", "conf": 0.xx, "lat_ms": int}
#     return {"label":"WAVE_STOP","conf":0.84,"lat_ms":0}


# If you don’t have a helper yet, move your current inference code into this:
def run_inference_on_paths(
    frames: List[str], context: Dict[str, Any]
) -> Dict[str, Any]:
    # (reuse your current /infer handler’s core; stub below)
    return {"label": "WAVE_STOP", "conf": 0.84, "lat_ms": 0}


class InferRequest(BaseModel):
    frames: List[str]
    context: Dict[str, Any] = {}


@app.post("/infer")
def infer(req: InferRequest):
    return run_inference_on_paths(req.frames, req.context)


# NEW: accept raw image upload
@app.post("/infer_bytes")
async def infer_bytes(
    file: UploadFile = File(...),
    context: Dict[str, Any] = Body(default_factory=dict),
):
    # read bytes
    data = await file.read()
    # optional: validate/readable
    try:
        Image.open(io.BytesIO(data)).verify()
    except Exception:
        return {"error": "Unsupported or corrupt image"}

    # write to a temp file so we can reuse the path-based pipeline
    with NamedTemporaryFile(delete=False, suffix=".jpg") as tmp:
        tmp.write(data)
        temp_path = tmp.name

    try:
        result = run_inference_on_paths([temp_path], context)
    finally:
        try:
            os.remove(temp_path)
        except OSError:
            pass

    return result
