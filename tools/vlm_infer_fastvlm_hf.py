import argparse, json, torch, sys
from PIL import Image
from decord import VideoReader, cpu
from transformers import AutoTokenizer, AutoModelForCausalLM, AutoImageProcessor

def pick_frame(video_path):
    vr = VideoReader(video_path, ctx=cpu(0))
    mid = max(0, (len(vr) - 1)//2)
    return Image.fromarray(vr[mid].asnumpy()).convert("RGB")

ap = argparse.ArgumentParser()
ap.add_argument("--model", default="apple/FastVLM-1.5B")
ap.add_argument("--clip", required=True)
ap.add_argument("--prompt", default="Describe the gesture briefly and return one label.")
ap.add_argument("--device", default="cpu", choices=["cpu","cuda"])
ap.add_argument("--max_new_tokens", type=int, default=64)
args = ap.parse_args()

device = "cuda" if (args.device=="cuda" and torch.cuda.is_available()) else "cpu"
dtype  = torch.bfloat16 if (device=="cuda" and torch.cuda.is_bf16_supported()) else (torch.float16 if device=="cuda" else torch.float32)

tokenizer = AutoTokenizer.from_pretrained(args.model, trust_remote_code=True, use_fast=False)
model = AutoModelForCausalLM.from_pretrained(args.model, trust_remote_code=True, torch_dtype=dtype).to(device)

# image -> CLIP image processor
image_processor = AutoImageProcessor.from_pretrained("openai/clip-vit-large-patch14")

img = pick_frame(args.clip)
pixel_values = image_processor(img, return_tensors="pt")["pixel_values"].to(device, dtype=dtype)

# *** STRING CHAT TEMPLATE WITH "<image>" ***
messages = [{"role":"user","content":"<image>\n" + args.prompt.strip()}]
input_ids = tokenizer.apply_chat_template(
    messages, add_generation_prompt=True, tokenize=True, return_tensors="pt"
).to(device)

if not isinstance(input_ids, torch.Tensor):
    print(json.dumps({"error":"tokenizer returned non-tensor input_ids"}))
    sys.exit(1)

# LLaVA-style expects a list of image tensors
images_arg = [pixel_values]

with torch.inference_mode():
    out = model.generate(input_ids=input_ids, images=images_arg, max_new_tokens=args.max_new_tokens)

text = tokenizer.decode(out[0], skip_special_tokens=True).strip()
print(json.dumps({"label": text, "rationale": text, "confidence": 0.0}))
