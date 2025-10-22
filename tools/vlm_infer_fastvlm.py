import os, sys, argparse, json, torch, warnings
warnings.filterwarnings("ignore")
fastvlm_src = os.environ.get("FASTVLM_SRC")
if fastvlm_src and fastvlm_src not in sys.path: sys.path.insert(0, fastvlm_src)

from decord import VideoReader, cpu
from fastvlm import FastVLMForConditionalGeneration, FastVLMProcessor

def load_frames(path, num=16):
    vr=VideoReader(path, ctx=cpu(0))
    idx=(torch.linspace(0, len(vr)-1, steps=num).long().tolist())
    return [vr[i].asnumpy() for i in idx]

ap=argparse.ArgumentParser()
ap.add_argument("--model_id", default="apple/FastVLM2-1.6B")
ap.add_argument("--clip", required=True)
ap.add_argument("--prompt", default="Describe the gesture briefly and give a single label.")
ap.add_argument("--device", default="cpu")
args=ap.parse_args()

dtype=torch.float16 if args.device.startswith("cuda") else torch.float32
model=FastVLMForConditionalGeneration.from_pretrained(args.model_id, torch_dtype=dtype)
proc=FastVLMProcessor.from_pretrained(args.model_id)
model.to(args.device)

frames=load_frames(args.clip, num=16)
inputs=proc(text=args.prompt, videos=frames, return_tensors="pt").to(args.device)
out=model.generate(**inputs, max_new_tokens=64)
text=proc.batch_decode(out, skip_special_tokens=True)[0].strip()
print(json.dumps({"label":text, "confidence":0.0, "rationale":text}))
