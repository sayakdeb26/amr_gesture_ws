#!/usr/bin/env python3
"""
metrics_logger_node.py

Non-invasive evaluation logger for the gesture control pipeline.
Subscribes to pipeline topics and writes episode-level metrics to CSV.

Topic Map (discovered from workspace):
  /intents_raw          (amr_interfaces/Intent)        - LSTM direct recognitions
  /lstm/unknown         (amr_interfaces/UnknownGesture) - Escalation triggers
  /vlm/confirm_request  (amr_interfaces/ConfirmRequest) - VLM results to kiosk
  /ui/confirm_reply     (amr_interfaces/ConfirmReply)   - Operator decisions
  /recorder/clip_ready  (std_msgs/String JSON)          - Clip paths

Correlation: Uses session_id field present in all amr_interfaces messages.
"""

import os
import csv
import json
import uuid
from datetime import datetime, timezone
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from std_msgs.msg import String
from amr_interfaces.msg import Intent, UnknownGesture, ConfirmRequest, ConfirmReply


@dataclass
class Episode:
    """Tracks a single gesture recognition episode."""
    episode_id: str
    session_id: str
    
    # Run annotations (set at init)
    mode: str = ""
    sensor_variant: str = ""
    environment_id: str = ""
    user_id: str = ""
    run_id: str = ""
    
    # Raw timestamps (unix seconds as float)
    t_lstm_out: Optional[float] = None
    t_escalate_trigger: Optional[float] = None
    t_vlm_out: Optional[float] = None
    t_kiosk_request: Optional[float] = None
    t_kiosk_reply: Optional[float] = None
    
    # Model outputs
    lstm_label: str = ""
    lstm_conf: float = 0.0
    escalated: bool = False
    escalation_reason: str = ""
    vlm_label: str = ""
    vlm_conf: float = 0.0
    vlm_status: str = ""  # ok | timeout | no_response | low_conf | parse_error
    
    # Human verification
    operator_decision: str = ""  # approve | reject | timeout
    final_label: str = ""
    timed_out: bool = False
    
    # Metadata
    clip_path: str = ""
    window_id: int = -1
    
    # Finalization state
    finalized: bool = False
    created_at: float = field(default_factory=lambda: datetime.now(timezone.utc).timestamp())


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__('metrics_logger_node')
        
        # Declare parameters
        self.declare_parameter('output_dir', '/home/sayak/amr_eval_logs')
        self.declare_parameter('file_prefix', 'run_metrics')
        self.declare_parameter('run_id', '')
        self.declare_parameter('mode', 'A')
        self.declare_parameter('sensor_variant', 'webcam')
        self.declare_parameter('environment_id', 'E1')
        self.declare_parameter('user_id', 'U1')
        self.declare_parameter('correlation_window_s', 2.0)
        self.declare_parameter('episode_timeout_s', 22.0)
        self.declare_parameter('flush_every_n_episodes', 1)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.run_id = self.get_parameter('run_id').value
        self.mode = self.get_parameter('mode').value
        self.sensor_variant = self.get_parameter('sensor_variant').value
        self.environment_id = self.get_parameter('environment_id').value
        self.user_id = self.get_parameter('user_id').value
        self.correlation_window_s = self.get_parameter('correlation_window_s').value
        self.episode_timeout_s = self.get_parameter('episode_timeout_s').value
        self.flush_every_n = self.get_parameter('flush_every_n_episodes').value
        
        # Auto-generate run_id if not provided
        if not self.run_id:
            self.run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Episode tracking
        self.episodes: Dict[str, Episode] = {}
        self.finalized_count = 0
        self.pending_flush_count = 0
        
        # Setup output
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(
            self.output_dir, 
            f"{self.file_prefix}_{self.run_id}.csv"
        )
        self._init_csv()
        
        # Subscriptions
        self.sub_intent = self.create_subscription(
            Intent, '/intents_raw', self.on_intent, 10)
        self.sub_unknown = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_unknown, 10)
        self.sub_confirm_req = self.create_subscription(
            ConfirmRequest, '/vlm/confirm_request', self.on_confirm_request, 10)
        self.sub_confirm_reply = self.create_subscription(
            ConfirmReply, '/ui/confirm_reply', self.on_confirm_reply, 10)
        self.sub_clip_ready = self.create_subscription(
            String, '/recorder/clip_ready', self.on_clip_ready, 10)
        
        # Timeout checker timer
        self.create_timer(1.0, self.check_timeouts)
        
        self.get_logger().info(
            f"Metrics Logger started. Output: {self.csv_path} | "
            f"Run: {self.run_id} | Mode: {self.mode} | Sensor: {self.sensor_variant}"
        )
    
    def _init_csv(self):
        """Initialize CSV with header row."""
        self.csv_columns = [
            'episode_id', 'session_id', 'window_id',
            'mode', 'sensor_variant', 'environment_id', 'user_id', 'run_id',
            't_lstm_out_unix', 't_lstm_out_iso',
            'lstm_label', 'lstm_conf',
            'escalated', 'escalation_reason',
            't_escalate_trigger_unix', 't_escalate_trigger_iso',
            't_vlm_out_unix', 't_vlm_out_iso',
            'vlm_label', 'vlm_conf', 'vlm_status',
            't_kiosk_request_unix', 't_kiosk_request_iso',
            't_kiosk_reply_unix', 't_kiosk_reply_iso',
            'operator_decision', 'final_label', 'timed_out',
            'hitl_latency_s', 'time_to_escalate_s', 'vlm_latency_s', 'end_to_end_s',
            'clip_path'
        ]
        
        # Write header if file doesn't exist
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_columns)
            self.get_logger().info(f"Created CSV: {self.csv_path}")
    
    def _get_or_create_episode(self, session_id: str) -> Episode:
        """Get existing episode or create new one."""
        if session_id not in self.episodes:
            ep = Episode(
                episode_id=str(uuid.uuid4())[:8],
                session_id=session_id,
                mode=self.mode,
                sensor_variant=self.sensor_variant,
                environment_id=self.environment_id,
                user_id=self.user_id,
                run_id=self.run_id
            )
            self.episodes[session_id] = ep
            self.get_logger().debug(f"New episode: {session_id}")
        return self.episodes[session_id]
    
    def _ros_time_to_unix(self, stamp) -> float:
        """Convert ROS builtin_interfaces/Time to unix timestamp."""
        return float(stamp.sec) + float(stamp.nanosec) / 1e9
    
    def _unix_to_iso(self, ts: Optional[float]) -> str:
        """Convert unix timestamp to ISO 8601 string."""
        if ts is None:
            return ""
        return datetime.fromtimestamp(ts, timezone.utc).isoformat()
    
    def _now_unix(self) -> float:
        """Get current time as unix timestamp."""
        return datetime.now(timezone.utc).timestamp()
    
    # ========== Callbacks ==========
    
    def on_intent(self, msg: Intent):
        """Handle direct LSTM intent (non-escalated path)."""
        session_id = msg.session_id
        if not session_id:
            return
        
        ep = self._get_or_create_episode(session_id)
        ep.t_lstm_out = self._ros_time_to_unix(msg.stamp)
        ep.lstm_label = msg.label
        ep.lstm_conf = msg.confidence
        
        # If source is "lstm" and not escalated, this is a direct recognition
        if msg.source == "lstm":
            ep.escalated = False
            ep.final_label = msg.label
            # Finalize immediately for non-escalated path
            self._finalize_episode(session_id)
        elif msg.source == "ui+vlm":
            # This is the final intent after VLM+operator approval
            # Don't finalize here - wait for confirm_reply
            pass
    
    def on_unknown(self, msg: UnknownGesture):
        """Handle LSTM unknown/low-confidence trigger (escalation start)."""
        session_id = msg.session_id
        if not session_id:
            return
        
        ep = self._get_or_create_episode(session_id)
        ep.t_lstm_out = self._ros_time_to_unix(msg.stamp)
        ep.t_escalate_trigger = self._now_unix()
        ep.lstm_label = msg.label
        ep.lstm_conf = msg.confidence
        ep.escalated = True
        ep.escalation_reason = "low_conf" if msg.confidence < 0.8 else "unknown"
        ep.window_id = msg.window_id
        
        self.get_logger().info(f"Escalation triggered: {session_id} ({msg.label} @ {msg.confidence:.2f})")
    
    def on_confirm_request(self, msg: ConfirmRequest):
        """Handle VLM result sent to kiosk."""
        session_id = msg.session_id
        if not session_id:
            return
        
        ep = self._get_or_create_episode(session_id)
        ep.t_vlm_out = self._ros_time_to_unix(msg.stamp)
        ep.t_kiosk_request = self._now_unix()
        ep.vlm_label = msg.candidate_label
        ep.vlm_conf = msg.candidate_conf
        ep.window_id = msg.window_id
        
        # Determine VLM status
        if msg.candidate_label == "PROCESSING...":
            ep.vlm_status = "processing"
        elif msg.candidate_label == "UNKNOWN":
            ep.vlm_status = "unknown"
        elif msg.candidate_conf < 0.5:
            ep.vlm_status = "low_conf"
        else:
            ep.vlm_status = "ok"
        
        self.get_logger().debug(f"VLM result: {session_id} → {msg.candidate_label}")
    
    def on_confirm_reply(self, msg: ConfirmReply):
        """Handle operator decision from kiosk."""
        session_id = msg.session_id
        if not session_id:
            return
        
        if session_id not in self.episodes:
            self.get_logger().warn(f"Reply for unknown session: {session_id}")
            return
        
        ep = self.episodes[session_id]
        ep.t_kiosk_reply = self._ros_time_to_unix(msg.stamp)
        
        if msg.approved:
            ep.operator_decision = "approve"
            ep.final_label = msg.final_label
        else:
            ep.operator_decision = "reject"
            ep.final_label = msg.final_label if msg.final_label else ""
        
        # Check if this was a timeout (empty final_label on reject often means timeout)
        if not msg.approved and not msg.final_label:
            ep.timed_out = True
            ep.operator_decision = "timeout"
        
        self._finalize_episode(session_id)
    
    def on_clip_ready(self, msg: String):
        """Handle clip path from recorder."""
        try:
            data = json.loads(msg.data)
            session_id = data.get('session_id', '')
            clip_path = data.get('clip_path', '')
            
            if session_id and session_id in self.episodes:
                self.episodes[session_id].clip_path = clip_path
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in clip_ready: {msg.data[:100]}")
    
    # ========== Finalization ==========
    
    def check_timeouts(self):
        """Check for timed-out episodes."""
        now = self._now_unix()
        timed_out = []
        
        for sid, ep in self.episodes.items():
            if ep.finalized:
                continue
            age = now - ep.created_at
            if age > self.episode_timeout_s:
                ep.timed_out = True
                if ep.escalated:
                    ep.operator_decision = "timeout"
                timed_out.append(sid)
        
        for sid in timed_out:
            self.get_logger().warn(f"Episode timed out: {sid}")
            self._finalize_episode(sid)
    
    def _finalize_episode(self, session_id: str):
        """Finalize episode and write to CSV."""
        if session_id not in self.episodes:
            return
        
        ep = self.episodes[session_id]
        if ep.finalized:
            return
        
        ep.finalized = True
        self.finalized_count += 1
        self.pending_flush_count += 1
        
        # Compute derived metrics
        hitl_latency = None
        if ep.t_kiosk_reply and ep.t_kiosk_request:
            hitl_latency = ep.t_kiosk_reply - ep.t_kiosk_request
        
        time_to_escalate = None
        if ep.t_escalate_trigger and ep.t_lstm_out:
            time_to_escalate = ep.t_escalate_trigger - ep.t_lstm_out
        
        vlm_latency = None
        if ep.t_vlm_out and ep.t_escalate_trigger:
            vlm_latency = ep.t_vlm_out - ep.t_escalate_trigger
        
        end_to_end = None
        if ep.escalated:
            if ep.t_kiosk_reply and ep.t_lstm_out:
                end_to_end = ep.t_kiosk_reply - ep.t_lstm_out
        else:
            # Non-escalated: just LSTM inference time (minimal)
            end_to_end = 0.0
        
        # Build row
        row = {
            'episode_id': ep.episode_id,
            'session_id': ep.session_id,
            'window_id': ep.window_id,
            'mode': ep.mode,
            'sensor_variant': ep.sensor_variant,
            'environment_id': ep.environment_id,
            'user_id': ep.user_id,
            'run_id': ep.run_id,
            't_lstm_out_unix': ep.t_lstm_out if ep.t_lstm_out else '',
            't_lstm_out_iso': self._unix_to_iso(ep.t_lstm_out),
            'lstm_label': ep.lstm_label,
            'lstm_conf': f"{ep.lstm_conf:.4f}" if ep.lstm_conf else '',
            'escalated': 1 if ep.escalated else 0,
            'escalation_reason': ep.escalation_reason,
            't_escalate_trigger_unix': ep.t_escalate_trigger if ep.t_escalate_trigger else '',
            't_escalate_trigger_iso': self._unix_to_iso(ep.t_escalate_trigger),
            't_vlm_out_unix': ep.t_vlm_out if ep.t_vlm_out else '',
            't_vlm_out_iso': self._unix_to_iso(ep.t_vlm_out),
            'vlm_label': ep.vlm_label,
            'vlm_conf': f"{ep.vlm_conf:.4f}" if ep.vlm_conf else '',
            'vlm_status': ep.vlm_status,
            't_kiosk_request_unix': ep.t_kiosk_request if ep.t_kiosk_request else '',
            't_kiosk_request_iso': self._unix_to_iso(ep.t_kiosk_request),
            't_kiosk_reply_unix': ep.t_kiosk_reply if ep.t_kiosk_reply else '',
            't_kiosk_reply_iso': self._unix_to_iso(ep.t_kiosk_reply),
            'operator_decision': ep.operator_decision,
            'final_label': ep.final_label,
            'timed_out': 1 if ep.timed_out else 0,
            'hitl_latency_s': f"{hitl_latency:.3f}" if hitl_latency is not None else '',
            'time_to_escalate_s': f"{time_to_escalate:.3f}" if time_to_escalate is not None else '',
            'vlm_latency_s': f"{vlm_latency:.3f}" if vlm_latency is not None else '',
            'end_to_end_s': f"{end_to_end:.3f}" if end_to_end is not None else '',
            'clip_path': ep.clip_path
        }
        
        # Write to CSV
        self._write_row(row)
        
        # Cleanup
        del self.episodes[session_id]
        
        self.get_logger().info(
            f"Episode finalized: {ep.session_id} | "
            f"{'ESCALATED' if ep.escalated else 'DIRECT'} | "
            f"Label: {ep.final_label or ep.lstm_label} | "
            f"Decision: {ep.operator_decision or 'N/A'}"
        )
    
    def _write_row(self, row: dict):
        """Write a row to CSV."""
        try:
            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=self.csv_columns)
                writer.writerow(row)
            
            if self.pending_flush_count >= self.flush_every_n:
                self.pending_flush_count = 0
                # Force flush by reopening (already closed above)
                
        except Exception as e:
            self.get_logger().error(f"CSV write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Write any pending episodes on shutdown
        for sid in list(node.episodes.keys()):
            node._finalize_episode(sid)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
