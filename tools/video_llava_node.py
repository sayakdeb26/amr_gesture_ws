import argparse, json, torch, re
from collections import Counter
from typing import List
from PIL import Image
from decord import VideoReader, cpu
from transformers import AutoProcessor, LlavaOnevisionForConditionalGeneration

DEFAULT_LABELS = [
    "thumbs_up","thumbs_down","open_palm","fist","peace_sign",
    "pointing","okay_sign","stop","wave","unknown"
]

PROMPT_TEMPLATE = """You are a precise gesture recognizer.
Given a single frame from a short human-gesture video, answer with a ONE-WORD label from this set:
{label_list}

If unsure, answer "unknown".
Only output the label word."""

def sample_frames(path: str, num: int) -> List[Image.Image]:
    vr = VideoReader(path, ctx=cpu(0))
    idxs = torch.linspace(0, len(vr)-1, steps=max(1, num)).long().tolist()
    return [Image.fromarray(vr[i].asnumpy()).convert("RGB") for i in idxs]

def normalize_label(text: str, labels: List[str]) -> str:
    t = text.strip().lower()
    # first word only; drop punctuation
    t = re.sub(r"[^a-z_]+", " ", t).split()
    t = t[0] if t else "unknown"
    return t if t in labels else "unknown"

def run_one_image(img: Image.Image, processor, model, device, dtype, labels, max_new_tokens: int):
    prompt = PROMPT_TEMPLATE.format(label_list=", ".join(labels))
    # OneVision accepts <image> token in the text + images=
    chat = f"USER: <image>\n{prompt}\nASSISTANT:"
    inputs = processor(text=chat, images=img, return_tensors="pt")
    # sanitize keys some processor versions add that generate() doesn't accept
    for k in ("batch_num_images","video_grid","video_frames"):
        inputs.pop(k, None)
    # move to device AFTER popping
    inputs = {k: (v.to(device, dtype=dtype) if isinstance(v, torch.Tensor) else v) for k,v in inputs.items()}

    with torch.inference_mode():
        out_ids = model.generate(**inputs, max_new_tokens=max_new_tokens)
    out = processor.batch_decode(out_ids, skip_special_tokens=True)[0]
    return normalize_label(out, labels), out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="llava-hf/llava-onevision-qwen2-0.5b-ov-hf")
    ap.add_argument("--clip", required=True)
    ap.add_argument("--device", default="cuda", choices=["cuda","cpu"])
    ap.add_argument("--num_frames", type=int, default=8)
    ap.add_argument("--labels", nargs="*", default=DEFAULT_LABELS)
    ap.add_argument("--max_new_tokens", type=int, default=16)
    args = ap.parse_args()

    device = "cuda" if (args.device=="cuda" and torch.cuda.is_available()) else "cpu"
    dtype  = (torch.bfloat16 if device=="cuda" and torch.cuda.is_bf16_supported()
              else torch.float16 if device=="cuda" else torch.float32)

    processor = AutoProcessor.from_pretrained(args.model)
    model = LlavaOnevisionForConditionalGeneration.from_pretrained(
        args.model, dtype=dtype, device_map=None
    ).to(device)

    frames = sample_frames(args.clip, args.num_frames)
    preds, raws = [], []
    for img in frames:
        y, raw = run_one_image(img, processor, model, device, dtype, args.labels, args.max_new_tokens)
        preds.append(y); raws.append(raw)

    vote = Counter(preds).most_common(1)[0][0] if preds else "unknown"
    print(json.dumps({
        "model": args.model,
        "device": device,
        "dtype": str(dtype),
        "frames": len(frames),
        "per_frame": preds,
        "raw": raws,
        "vote": vote
    }, ensure_ascii=False))
if __name__ == "__main__":
    main()
