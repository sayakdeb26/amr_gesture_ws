import argparse, json, torch
from PIL import Image
from transformers import AutoTokenizer, AutoModelForCausalLM
from decord import VideoReader, cpu

def pick_dtype(device: str):
    if device.startswith("cuda") and torch.cuda.is_available():
        return torch.bfloat16 if torch.cuda.is_bf16_supported() else torch.float16
    return torch.float32

def load_frames_as_pil(path, num=1):
    vr = VideoReader(path, ctx=cpu(0))
    if num <= 1:
        idxs = [int((len(vr)-1)//2)]
    else:
        import numpy as np
        idxs = torch.linspace(0, len(vr)-1, steps=num).long().tolist()
    return [Image.fromarray(vr[i].asnumpy()).convert("RGB") for i in idxs]

ap = argparse.ArgumentParser()
ap.add_argument("--model", default="apple/FastVLM-1.5B")
ap.add_argument("--clip", required=True)
ap.add_argument("--device", default="cuda")
ap.add_argument("--num_frames", type=int, default=1)
ap.add_argument("--max_new_tokens", type=int, default=64)
ap.add_argument("--prompt", default="Describe the gesture briefly and return a single label name.")
args = ap.parse_args()

dtype = pick_dtype(args.device)

tok = AutoTokenizer.from_pretrained(args.model, trust_remote_code=True)
model = AutoModelForCausalLM.from_pretrained(args.model, trust_remote_code=True, torch_dtype=dtype)
model.to(args.device)

if args.device.startswith("cuda"):
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.set_float32_matmul_precision("high")

user_text = f"<image>\n{args.prompt}"
inputs = tok([user_text], return_tensors="pt").to(args.device)

images = load_frames_as_pil(args.clip, num=args.num_frames)
images_arg = images[0] if len(images)==1 else images

with torch.inference_mode():
    out = model.generate(**inputs, images=images_arg, max_new_tokens=args.max_new_tokens)

text = tok.batch_decode(out, skip_special_tokens=True)[0].strip()
print(json.dumps({
    "label": text,
    "rationale": text,
    "device": args.device,
    "dtype": str(dtype).replace("torch.", ""),
}))
