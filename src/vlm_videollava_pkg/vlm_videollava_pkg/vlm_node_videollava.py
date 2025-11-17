#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
LLaVA-NeXT-Video backend for /vlm/infer:

- Service:  /vlm/infer
- Type:     vlm_interfaces/srv/InferClip
- Input:    path to a video clip (mp4) + optional label_hint
- Output:   gesture label (from your 12-class set), confidence, rationale

Internally:
- Uses HuggingFace transformers LlavaNextVideoForConditionalGeneration
  with LlavaNextVideoProcessor on top of "llava-hf/LLaVA-NeXT-Video-7B-hf".
"""

import os
import sys

# Ensure user site-packages (where `av` is installed) are on sys.path
user_site = os.path.expanduser("~/.local/lib/python3.10/site-packages")
if user_site not in sys.path:
    sys.path.append(user_site)

import rclpy
from rclpy.node import Node

import torch
from transformers import (
    LlavaNextVideoForConditionalGeneration,
    LlavaNextVideoProcessor,
)

from vlm_interfaces.srv import InferClip


class VideoLlavaNode(Node):
    """
    ROS2 service node wrapping LLaVA-NeXT-Video-7B as a gesture classifier.
    """

    def __init__(self):
        super().__init__("vlm_node")

        # Parameters
        self.declare_parameter("model_id", "llava-hf/LLaVA-NeXT-Video-7B-hf")
        self.declare_parameter("num_frames", 6)
        self.declare_parameter("device", "cpu")  # "cuda" or "cpu"

        self.model_id = (
            self.get_parameter("model_id").get_parameter_value().string_value
        )
        self.num_frames = (
            self.get_parameter("num_frames").get_parameter_value().integer_value
        )
        self.device = (
            self.get_parameter("device").get_parameter_value().string_value
        )

        self.get_logger().info(
            f"[VideoLLaVA] Initializing with model_id={self.model_id}, "
            f"num_frames={self.num_frames}, device={self.device}"
        )

        # Load LLaVA-NeXT-Video model + processor
        self._init_model()

        # Service server
        self.srv = self.create_service(
            InferClip,
            "/vlm/infer",
            self.handle_infer,
        )
        self.get_logger().info("[VideoLLaVA] /vlm/infer service is ready")

    # ------------------------------------------------------------------ #
    #  Model init
    # ------------------------------------------------------------------ #
    def _init_model(self):
        """
        Load LLaVA-NeXT-Video model & processor.
        """

        use_cuda = False
        torch_dtype = torch.float32
        device_map = None


        self.get_logger().info(
            f"[VideoLLaVA] Loading LLaVA-NeXT-Video from {self.model_id} "
            f"(dtype={torch_dtype}, device_map={device_map})"
        )

        # Model
        self.model = LlavaNextVideoForConditionalGeneration.from_pretrained(
            self.model_id,
            torch_dtype=torch_dtype,
            device_map=device_map,
            low_cpu_mem_usage=True,
        )

        # Processor (wraps video processor + image processor + tokenizer)
        self.processor = LlavaNextVideoProcessor.from_pretrained(self.model_id)

        # Recommended by HF docs for batched generation
        # (left-padding gives more accurate results). :contentReference[oaicite:0]{index=0}
        self.processor.tokenizer.padding_side = "left"

        self.model.eval()
        self.get_logger().info("[VideoLLaVA] Model and processor loaded")

    # ------------------------------------------------------------------ #
    #  Service handler
    # ------------------------------------------------------------------ #
    def handle_infer(self, request: InferClip.Request, response: InferClip.Response):
        clip_path = (request.clip_path or "").strip()
        label_hint = (request.label_hint or "").strip()

        if not clip_path or not os.path.exists(clip_path):
            msg = f"[VideoLLaVA] Clip not found: '{clip_path}'"
            self.get_logger().warn(msg)
            response.label = "UNKNOWN"
            response.confidence = 0.0
            response.rationale = msg
            return response

        user_prompt = self._build_prompt(label_hint)

        try:
            label, confidence, rationale = self._infer_clip(clip_path, user_prompt)
        except Exception as e:
            msg = f"[VideoLLaVA] Exception during inference: {e}"
            self.get_logger().error(msg)
            response.label = "UNKNOWN"
            response.confidence = 0.0
            response.rationale = msg
            return response

        response.label = label
        response.confidence = float(confidence)
        response.rationale = rationale
        return response

    # ------------------------------------------------------------------ #
    #  Prompt + inference
    # ------------------------------------------------------------------ #
    def _build_prompt(self, label_hint: str) -> str:
        base = (
            "You will see a short video clip of a person performing a hand gesture. "
            "Respond with a single gesture label using the format WORD or WORD_WORD. "
            "Examples: ROLL_FWD, HAND_SHAKE, OPEN_PALM, STOP, PINCH_IN, SWIPE_LEFT.\n"
            "Do not output full sentences. Only output the label token."
        )
        if label_hint:
            base += f"\nHint: {label_hint}\n"
        base += "\nLabel:"
        return base


    def _infer_clip(self, clip_path: str, user_prompt: str):
        """
        Run LLaVA-NeXT-Video on the given clip path + prompt.

        Returns:
          (label, confidence, rationale)
        """

        conversation = [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_prompt},
                    {"type": "video", "path": clip_path},
                ],
            },
        ]

        inputs = self.processor.apply_chat_template(
            conversation,
            num_frames=self.num_frames,
            add_generation_prompt=True,
            tokenize=True,
            return_dict=True,
            padding=True,
            return_tensors="pt",
        )

        # move tensors to model device if needed
        for k, v in inputs.items():
            if isinstance(v, torch.Tensor):
                inputs[k] = v.to(self.model.device)

        with torch.no_grad():
            generate_ids = self.model.generate(
                **inputs,
                max_new_tokens=64,
                do_sample=False,
            )

        decoded_full = self.processor.batch_decode(
            generate_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=True,
        )[0].strip()

        # 🔑 only use the assistant's final answer text for label mapping
        answer_text = self._extract_assistant_answer(decoded_full)

        label, confidence = self._canonicalize_label(answer_text)
        rationale = f"LLaVA-NeXT-Video raw answer: {decoded_full}"
        return label, confidence, rationale


    # ------------------------------------------------------------------ #
    #  Text → gesture label mapping
    # ------------------------------------------------------------------ #
   
    def _extract_assistant_answer(self, text: str) -> str:
        """
        Given the full decoded conversation, extract just the assistant's final answer.
        Example input:
          'USER: ... Label: ASSISTANT: ROLL_FWD'
        We want: 'ROLL_FWD'
        """
        lower = text.lower()
        if "assistant:" in lower:
            # split on last 'assistant:' to be safe
            parts = text.split("ASSISTANT:")
            answer = parts[-1]
        else:
            # fall back to the whole text
            answer = text
        return answer.strip()



    def _canonicalize_label(self, text: str):
        """
        Convert arbitrary LLaVA output into a clean label.

        Rules:
        - Extract only uppercase-word-like tokens (A_Z or A_Z_A_Z)
        - Allow ANY label in WORD, WORD_WORD, WORD_WORD_WORD format
        - If no valid label detected → return UNKNOWN
        """

        if not text:
            return "UNKNOWN", 0.0

        # Normalize
        t = text.strip()

        # If LLaVA already returned clean token (e.g. ROLL_FWD)
        if "_" in t and t.replace("_", "").isalpha():
            return t.upper(), 0.9

        # Otherwise split into tokens and find first "label-like" token
        import re
        # Accept words OR WORD_WORD OR WORD_WORD_WORD
        pattern = r"[A-Za-z]+(?:_[A-Za-z]+)*"

        matches = re.findall(pattern, t)
        if matches:
            # Take the first candidate as label
            label = matches[0].upper()
            return label, 0.8

        # Nothing matched
        return "UNKNOWN", 0.0



def main(args=None):
    rclpy.init(args=args)
    node = VideoLlavaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

