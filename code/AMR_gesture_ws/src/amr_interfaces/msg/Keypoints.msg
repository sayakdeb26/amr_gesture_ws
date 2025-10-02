# Timestamp and frame
builtin_interfaces/Time stamp
string frame_id

# 2D keypoints (flattened [x0,y0, x1,y1, ...]) normalized [0,1] or pixels
float32[] keypoints2d

# 3D keypoints (flattened [x0,y0,z0, x1,y1,z1, ...]) if available
float32[] keypoints3d

# Optional: which model produced these (mediapipe, openpose, etc.)
string source

# What set of keypoints (hands|full_body|hands+pose)
string modality
