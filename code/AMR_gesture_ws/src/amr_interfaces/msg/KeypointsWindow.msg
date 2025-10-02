builtin_interfaces/Time stamp
uint32 frames
uint32 joints_per_frame     # e.g., 21 hand joints * 2 hands = 42
float32[] data              # row-major: [frame0_j0x, frame0_j0y, frame0_j0z, ..., frameF_jJz]
string source               # e.g., 'mediapipe'
