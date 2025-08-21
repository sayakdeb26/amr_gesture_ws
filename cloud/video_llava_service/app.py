from fastapi import FastAPI
from pydantic import BaseModel
import time
from .inference import load_model, infer_one

app = FastAPI()
model = load_model()


class Payload(BaseModel):
    frames: list[str]
    context: dict | None = None


@app.post("/infer")
def infer(payload: Payload):
    t0 = time.time()
    label, conf = infer_one(model, payload.frames, payload.context)
    lat_ms = int((time.time() - t0) * 1000)
    return {"label": label, "conf": conf, "lat_ms": lat_ms}
