#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vlm_interfaces.srv import InferClip
import torch
from transformers import VideoLlavaProcessor, VideoLlavaForConditionalGeneration
import av
import numpy as np

class VideoLLaVANode(Node):
    def __init__(self):
        super().__init__('video_llava_node')
        
        self.model_id = "LanguageBind/Video-LLaVA-7B-hf"
        self.device = "cpu"
        
        self.get_logger().info(f"Loading Video-LLaVA model: {self.model_id} on {self.device}...")
        try:
            self.processor = VideoLlavaProcessor.from_pretrained(self.model_id)
            self.model = VideoLlavaForConditionalGeneration.from_pretrained(
                self.model_id, 
                dtype=torch.float32,
                device_map={"": "cpu"},
                low_cpu_mem_usage=True
            )
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model = None

        self.srv = self.create_service(InferClip, '/vlm/infer', self.infer_callback)
        self.get_logger().info('Video-LLaVA Service Ready')

    def read_video_pyav(self, container, indices):
        frames = []
        container.seek(0)
        start_index = indices[0]
        end_index = indices[-1]
        for i, frame in enumerate(container.decode(video=0)):
            if i > end_index:
                break
            if i >= start_index and i in indices:
                frames.append(frame)
        return np.stack([x.to_ndarray(format="rgb24") for x in frames])

    def infer_callback(self, request, response):
        self.get_logger().info(f"Processing clip: {request.clip_path}")
        
        if not self.model:
            self.get_logger().error("Model is None! Returning ERROR.")
            response.label = "ERROR"
            response.confidence = 0.0
            response.rationale = "Model not loaded"
            return response

        try:
            container = av.open(request.clip_path)
            total_frames = container.streams.video[0].frames
            # Sample frames centered around median (max 32 frames to stay within token limit)
            max_frames = 8  # Empirically determined to stay under 4096 token limit
            if total_frames <= max_frames:
                indices = np.arange(total_frames)
            else:
                median_idx = total_frames // 2
                half = max_frames // 2
                start = max(0, median_idx - half)
                end = min(total_frames, median_idx + half)
                indices = np.arange(start, end)
            video = self.read_video_pyav(container, indices)
            
            prompt = "USER: <video>\nWhat gesture is the person performing? Choose from: STOP_SIGN, SWIPE_DOWN, SWIPE_LEFT, SWIPE_RIGHT, SWIPE_UP, THUMB_UP, THUMB_DOWN, ZOOM_IN, ZOOM_OUT. Output only the label. ASSISTANT:"
            
            self.get_logger().info(f"Step 1: Video loaded, {len(video)} frames")
            inputs = self.processor(text=prompt, videos=video, return_tensors="pt").to(self.device, self.model.dtype)
            
            self.get_logger().info("Step 2: Inputs processed, calling model.generate()...")
            out = self.model.generate(**inputs, max_new_tokens=20)
            
            self.get_logger().info("Step 3: Generation complete, decoding...")
            generated_text = self.processor.batch_decode(out, skip_special_tokens=True)[0].strip()
            self.get_logger().info(f"Step 4: Decoded text: {generated_text}")
            
            # Parse response
            known_labels = ["STOP_SIGN", "SWIPE_DOWN", "SWIPE_LEFT", "SWIPE_RIGHT", "SWIPE_UP", "THUMB_UP", "THUMB_DOWN", "ZOOM_IN", "ZOOM_OUT"]
            found_label = "UNKNOWN"
            
            if "ASSISTANT:" in generated_text:
                generated_text = generated_text.split("ASSISTANT:")[-1].strip()
            
            text_upper = generated_text.upper()
            for label in known_labels:
                if label in text_upper:
                    found_label = label
                    break

            response.label = found_label
            response.confidence = 0.8
            response.rationale = generated_text
            
            self.get_logger().info(f"VLM Output: {generated_text} -> {found_label}")

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            response.label = "ERROR"
            response.confidence = 0.0
            response.rationale = str(e)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VideoLLaVANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
