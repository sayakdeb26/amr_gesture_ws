import os
import json
import shutil
import pathlib
from datetime import datetime

import rclpy
from rclpy.node import Node

from amr_interfaces.srv import (
    UpsertGesture,
    SaveMapping,
    SwitchProfile,
    ReloadRepo,
)
from amr_interfaces.msg import Diagnostics


def _expand(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


class RepoManager(Node):
    def __init__(self) -> None:
        super().__init__("repo_manager")

        # config/env
        self.repo_root = _expand(os.environ.get("GESTURE_REPO_DIR", "./data/repo"))
        self.active_profile = os.environ.get("GESTURE_ACTIVE_PROFILE", "base")

        self.get_logger().info(f"Repo root: {self.repo_root}")
        self.get_logger().info(f"Active profile: {self.active_profile}")

        # ensure dirs
        pathlib.Path(self.profile_dir).mkdir(parents=True, exist_ok=True)

        # state
        self._gestures: dict = {}
        self._mappings: dict = {}

        # services
        self.srv_upsert = self.create_service(
            UpsertGesture, "repo/upsert_gesture", self.handle_upsert
        )
        self.srv_mapping = self.create_service(
            SaveMapping, "repo/save_mapping", self.handle_save_mapping
        )
        self.srv_switch = self.create_service(
            SwitchProfile, "repo/switch_profile", self.handle_switch_profile
        )
        self.srv_reload = self.create_service(
            ReloadRepo, "repo/reload_repo", self.handle_reload
        )

        # diagnostics (optional)
        self.diag_pub = self.create_publisher(Diagnostics, "/diagnostics", 10)
        self.timer = self.create_timer(5.0, self.tick_diagnostics)

        # load initial
        self.load_profile(self.active_profile)

    @property
    def profile_dir(self) -> str:
        return os.path.join(self.repo_root, "profiles", self.active_profile)

    def _json_read(self, path: str):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return None

    def _json_atomic_write(self, path: str, payload: dict) -> None:
        tmp = f"{path}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        shutil.move(tmp, path)

    def _now_str(self) -> str:
        return f"{datetime.utcnow().isoformat()}Z"

    def load_profile(self, profile: str) -> None:
        self.active_profile = profile
        gd_path = os.path.join(self.repo_root, "profiles", profile, "gestures.json")
        mp_path = os.path.join(self.repo_root, "profiles", profile, "mappings.json")
        pathlib.Path(os.path.dirname(gd_path)).mkdir(parents=True, exist_ok=True)

        gestures = self._json_read(gd_path) or {
            "version": 1,
            "profile": profile,
            "updated_at": "",
            "gestures": {},
        }
        mappings = self._json_read(mp_path) or {
            "version": 1,
            "profile": profile,
            "updated_at": "",
            "mappings": {},
        }
        self._gestures = gestures
        self._mappings = mappings
        self.get_logger().info(
            f"Loaded profile '{profile}' with "
            f"{len(self._gestures.get('gestures', {}))} gestures, "
            f"{len(self._mappings.get('mappings', {}))} mappings."
        )

    def save_current(self) -> None:
        gd_path = os.path.join(self.profile_dir, "gestures.json")
        mp_path = os.path.join(self.profile_dir, "mappings.json")
        self._gestures["updated_at"] = self._now_str()
        self._mappings["updated_at"] = self._now_str()
        self._json_atomic_write(gd_path, self._gestures)
        self._json_atomic_write(mp_path, self._mappings)

    # Services
    def handle_upsert(
        self, req: UpsertGesture.Request, res: UpsertGesture.Response
    ) -> UpsertGesture.Response:
        ns = getattr(req, "repo_namespace", "base").strip() or "base"
        if ns != self.active_profile:
            self.get_logger().warn(
                f"Upsert into '{ns}' while active is '{self.active_profile}'. Switching."
            )
            self.load_profile(ns)
        try:
            blob = json.loads(req.json_blob) if req.json_blob else {}
        except Exception as exc:
            res.ok = False
            res.message = f"invalid json_blob: {exc}"
            return res

        gset = self._gestures.setdefault("gestures", {})
        key = req.label or req.gesture_id
        gset[key] = {"label": key, "gesture_id": req.gesture_id, **blob}
        self.save_current()
        res.ok = True
        res.message = f"upserted '{key}' into profile '{self.active_profile}'"
        self.get_logger().info(res.message)
        return res

    def handle_save_mapping(
        self, req: SaveMapping.Request, res: SaveMapping.Response
    ) -> SaveMapping.Response:
        mset = self._mappings.setdefault("mappings", {})
        try:
            params = json.loads(req.action_params) if req.action_params else {}
        except Exception as exc:
            res.ok = False
            res.message = f"invalid action_params json: {exc}"
            return res

        mset[req.gesture_id] = {
            "action_topic": req.action_topic,
            "action_type": req.action_type,
            "action_params": params,
        }
        self.save_current()
        res.ok = True
        res.message = f"mapping saved for '{req.gesture_id}'"
        self.get_logger().info(res.message)
        return res

    def handle_switch_profile(
        self, req: SwitchProfile.Request, res: SwitchProfile.Response
    ) -> SwitchProfile.Response:
        target = req.profile_id.strip() or "base"
        self.load_profile(target)
        self.save_current()
        res.ok = True
        res.message = f"active profile: '{self.active_profile}'"
        self.get_logger().info(res.message)
        return res

    def handle_reload(
        self, req: ReloadRepo.Request, res: ReloadRepo.Response
    ) -> ReloadRepo.Response:
        del req  # unused
        self.load_profile(self.active_profile)
        res.ok = True
        res.message = f"reloaded profile '{self.active_profile}'"
        self.get_logger().info(res.message)
        return res

    def tick_diagnostics(self) -> None:
        try:
            msg = Diagnostics()
            now = self.get_clock().now().to_msg()
            msg.stamp = now
            msg.conf_static = 0.0
            msg.conf_reflex = 0.0
            msg.conf_vlm = 0.0
            msg.vlm_latency_ms = 0
            msg.arbiter_state = f"profile:{self.active_profile}"
            msg.last_action = ""
            msg.note = (
                f"gestures={len(self._gestures.get('gestures', {}))}, "
                f"mappings={len(self._mappings.get('mappings', {}))}"
            )
            self.diag_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"diag error: {exc}")


def main() -> None:
    rclpy.init()
    node = RepoManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
