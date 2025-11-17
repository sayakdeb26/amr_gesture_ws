# vlm_interfaces/msg/ConfirmRequest.msg

string session_id
string window_id

string candidate_label
float32 candidate_confidence

# Path to recorded clip on disk
string clip_path

# OPTIONAL: middle frame of the clip as base64 (JPEG/PNG)
string preview_frame_b64

