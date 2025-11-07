#!/usr/bin/env python3
import os, cv2, torch
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from vlm_interfaces.srv import InferClip
from transformers import AutoProcessor, AutoTokenizer, AutoModelForCausalLM

MODEL_ID = os.getenv("VLM_MODEL_ID", "apple/FastVLM-1.5B-fp16")
FRAMES_TO_SAMPLE = 5
MAX_NEW_TOKENS = 24
PROMPT = "Classify the human hand gesture in this frame with a concise single label."

class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_node')
        self.get_logger().info(f"Loading {MODEL_ID} … (first run may download weights)")
        torch.backends.cuda.matmul.allow_tf32 = True
        trust = True
        self.processor = AutoProcessor.from_pretrained(MODEL_ID, trust_remote_code=trust)
        self.tokenizer  = AutoTokenizer.from_pretrained(MODEL_ID, trust_remote_code=trust)
        dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        self.model = AutoModelForCausalLM.from_pretrained(
            MODEL_ID, torch_dtype=dtype, device_map="auto", trust_remote_code=trust
        )
        self.device = next(self.model.parameters()).device
        self.get_logger().info(f"FastVLM loaded on device: {self.device}")
        self.srv = self.create_service(InferClip, '/vlm/infer', self.handle_infer)
        self.get_logger().info('VLM service ready at /vlm/infer')

    def handle_infer(self, req: InferClip.Request, resp: InferClip.Response):
        clip_path = (req.clip_path or "").strip()
        label_hint = (req.label_hint or "").strip()
        if not (clip_path and os.path.exists(clip_path)):
            resp.label = "UNKNOWN"; resp.confidence = 0.0; resp.rationale = f"Clip not found: {clip_path}"; return resp
        frames = self._sample_frames(clip_path, FRAMES_TO_SAMPLE)
        if not frames:
            resp.label = "UNKNOWN"; resp.confidence = 0.0; resp.rationale = "Could not decode frames."; return resp
        preds = []
        for bgr in frames:
            rgb = bgr[:, :, ::-1]
            text = self._infer_image(rgb, PROMPT)
            if text: preds.append(text.strip())
        if not preds:
            resp.label = "UNKNOWN"; resp.confidence = 0.0; resp.rationale = "No output."; return resp
        final_label, conf = self._aggregate(preds)
        if label_hint and label_hint.lower() in [p.lower() for p in preds]:
            conf = min(0.99, conf + 0.05)
        resp.label, resp.confidence = final_label, float(conf)
        resp.rationale = f"Frame votes={preds}; chosen='{final_label}'"
        self.get_logger().info(f"FastVLM → {final_label} (conf={conf:.2f})")
        return resp

    def _sample_frames(self, path: str, k: int) -> List:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened(): return []
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
        idxs = list(range(min(k, total))) if not total or k >= total else [int((i+1)*total/(k+1)) for i in range(k)]
        frames=[]
        for i in idxs:
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, f = cap.read()
            if ok and f is not None: frames.append(f)
        cap.release(); return frames

    @torch.inference_mode()
    def _infer_image(self, rgb, prompt: str) -> str:
        inputs = self.processor(images=rgb, text=prompt, return_tensors="pt").to(self.device)
        out_ids = self.model.generate(**inputs, max_new_tokens=MAX_NEW_TOKENS)
        texts = self.tokenizer.batch_decode(out_ids, skip_special_tokens=True)
        if texts:
            t = texts[0].strip()
            if t.lower().startswith(prompt.lower()):
                t = t[len(prompt):].strip(" :\n")
            return t.splitlines()[0].strip()
        return ""

    def _aggregate(self, preds: List[str]) -> Tuple[str, float]:
        from collections import Counter
        c = Counter([p.lower() for p in preds if p])
        label, votes = c.most_common(1)[0]
        conf = votes / max(1, len(preds))
        conf = 0.98 * conf + 0.01
        return label, float(conf)

def main():
    rclpy.init(); node = VLMNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
