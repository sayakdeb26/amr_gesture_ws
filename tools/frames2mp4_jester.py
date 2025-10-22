import os, json, argparse, subprocess, sys
from pathlib import Path
import pandas as pd
from collections import defaultdict

PATTERNS = ["%06d.jpg", "%05d.jpg", "%04d.jpg", "%d.jpg"]

def find_pattern(folder: Path):
    # detect zero-padding by checking existing files
    names = {p.name for p in folder.glob("*.jpg")}
    for pad in (6,5,4):
        if f"{1:0{pad}d}.jpg" in names:
            return f"{folder}/%0{pad}d.jpg"
    # fallback
    if "1.jpg" in names: return f"{folder}/%d.jpg"
    return None

def encode(folder: Path, out_mp4: Path, fps=10, size="320x240", crf=28, preset="veryfast"):
    patt = find_pattern(folder)
    if not patt:
        print(f"[skip] no JPG sequence in {folder}", file=sys.stderr)
        return False
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg","-loglevel","error","-y",
        "-framerate", str(fps),
        "-i", patt,
        "-s", size,
        "-c:v","libx264","-preset",preset,"-crf",str(crf),
        "-pix_fmt","yuv420p",
        str(out_mp4),
    ]
    try:
        subprocess.check_call(cmd)
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ffmpeg fail] {folder} -> {out_mp4} ({e})", file=sys.stderr)
        return False

def run_one_split(split_root: Path, split_csv: Path, out_root: Path, label_map: dict, max_per_class: int):
    df = pd.read_csv(split_csv)
    # column names in your CSV preview: video_id, label
    per_class = defaultdict(int)
    made = 0
    for _, row in df.iterrows():
        label_full = str(row["label"])
        mapped = label_map.get(label_full)
        if not mapped:
            continue  # label not used in Phase-1
        if max_per_class and per_class[mapped] >= max_per_class:
            continue
        vid = str(row["video_id"])
        folder = split_root / vid
        if not folder.is_dir():
            continue
        out_mp4 = out_root / mapped / f"{vid}.mp4"
        if out_mp4.exists():
            per_class[mapped] += 1
            made += 1
            continue
        ok = encode(folder, out_mp4)
        if ok:
            per_class[mapped] += 1
            made += 1
            if made % 25 == 0:
                print(f"...made {made} files", flush=True)
    print(f"Done {split_root.name}: {made} mp4s | per class: {dict(per_class)}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="~/datasets/jester")
    ap.add_argument("--out_root", required=True, help="~/datasets/jester_mp4")
    ap.add_argument("--label_map", required=True, help="JSON mapping Jester label -> our label")
    ap.add_argument("--max_per_class", type=int, default=0, help="cap per mapped class (0 = no cap)")
    args = ap.parse_args()

    root = Path(os.path.expanduser(args.root))
    out_root = Path(os.path.expanduser(args.out_root))
    label_map = json.load(open(os.path.expanduser(args.label_map), "r"))

    # process Train + Validation only (Test has no labels)
    for split in ("Train","Validation"):
        split_root = root / split
        split_csv  = root / f"{split}.csv"
        if not split_root.is_dir() or not split_csv.exists():
            print(f"[warn] missing {split_root} or {split_csv}, skipping")
            continue
        run_one_split(split_root, split_csv, out_root, label_map, args.max_per_class)

if __name__ == "__main__":
    main()
