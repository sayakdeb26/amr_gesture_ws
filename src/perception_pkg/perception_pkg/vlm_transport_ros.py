import rclpy
from vlm_interfaces.srv import InferClip

class VLMClientROS:
    def __init__(self, node):
        self._node = node
        self._client = node.create_client(InferClip, 'vlm/infer')
        if not self._client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error('VLM ROS service /vlm/infer not available')

    def infer(self, clip_path: str, label_hint: str = ''):
        req = InferClip.Request()
        req.clip_path = clip_path
        req.label_hint = label_hint or ''
        fut = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut)
        resp = fut.result()
        if resp is None:
            raise RuntimeError('VLM service call failed')
        return {
            "label": resp.label,
            "confidence": float(resp.confidence),
            "rationale": resp.rationale,
            "latency_ms": int(resp.latency_ms)
        }
