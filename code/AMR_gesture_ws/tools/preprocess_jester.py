import os, cv2, json, argparse, random, numpy as np
from pathlib import Path
from tqdm import tqdm

FPS=10; WIN=30; STRIDE=3
WRIST=0; INDEX_MCP=5
FEATS=84  # 2 hands * 21 joints * (x,y)

def init_hands():
    import mediapipe as mp
    return mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2,
                                    min_detection_confidence=0.5, min_tracking_confidence=0.5)

def wrist_center_scale(hand):
    if hand is None:
        return np.zeros((21,2), dtype=np.float32)
    H=hand.astype(np.float32).copy()
    base=H[WRIST]
    H-=base
    scale=np.linalg.norm(H[INDEX_MCP]) if np.isfinite(H[INDEX_MCP]).all() else 0.0
    if not np.isfinite(scale) or scale<1e-3: scale=1e-3
    H/=scale
    return np.clip(H, -3.0, 3.0)

def feats_from_frame(bgr, hp):
    h,w,_=bgr.shape
    rgb=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    res=hp.process(rgb)
    L=None; R=None
    if res.multi_hand_landmarks and res.multi_handedness:
        for lm,hd in zip(res.multi_hand_landmarks, res.multi_handedness):
            pts=np.array([[p.x*w, p.y*h] for p in lm.landmark], dtype=np.float32)  # image coords
            if hd.classification[0].label.lower()=='left':
                L=pts
            else:
                R=pts
    Ln=wrist_center_scale(L); Rn=wrist_center_scale(R)
    return np.concatenate([Ln.reshape(-1), Rn.reshape(-1)], 0)  # (84,)

def iter_sampled_frames(path, target_fps=FPS):
    cap=cv2.VideoCapture(str(path))
    if not cap.isOpened(): return
    src_fps=cap.get(cv2.CAP_PROP_FPS) or 30.0
    if not src_fps or not np.isfinite(src_fps): src_fps=30.0
    step=max(1, int(round(src_fps/target_fps)))
    i=0
    while True:
        ok, frame=cap.read()
        if not ok: break
        if i%step==0: yield frame
        i+=1
    cap.release()

def windows(seq, win=WIN, stride=STRIDE):
    n=len(seq)
    for s in range(0, max(0, n-win)+1, stride):
        yield np.stack(seq[s:s+win], axis=0)  # (win,84)

def collect_files(root):
    root=Path(root)
    out=[]
    for cls_dir in sorted([p for p in root.iterdir() if p.is_dir()]):
        label = cls_dir.name
        for vid in cls_dir.rglob("*"):
            if vid.suffix.lower() in (".mp4",".avi",".mov",".mkv",".gif"):
                out.append((vid, label))
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--data_root", required=True, help="Jester root with subfolders per original label")
    ap.add_argument("--label_map", required=True, help="JSON mapping: original_label -> your_label")
    ap.add_argument("--out_root", required=True)
    ap.add_argument("--val_ratio", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=42)
    args=ap.parse_args()

    with open(args.label_map,'r') as f: j2y=json.load(f)

    files=collect_files(args.data_root)
    # keep only mapped classes
    files=[(p,lbl) for (p,lbl) in files if lbl in j2y]
    if not files:
        raise SystemExit("No videos found after applying label_map. Check --data_root and label names.")

    random.Random(args.seed).shuffle(files)
    k=int((1.0-args.val_ratio)*len(files))
    train_files, val_files = files[:k], files[k:]

    # target class order (phase-1 preference)
    target_classes=sorted(set(j2y.values()))
    phase1=['wave_stop','follow_me','go_ahead','point','no_gesture']
    target_classes=[c for c in phase1 if c in target_classes] + [c for c in target_classes if c not in phase1]
    cls_to_idx={c:i for i,c in enumerate(target_classes)}

    out_root=Path(args.out_root); (out_root/'windows'/'train').mkdir(parents=True, exist_ok=True)
    (out_root/'windows'/'val').mkdir(parents=True, exist_ok=True)

    def process_split(split, outdir, collect_stats):
        import mediapipe as mp; hp=init_hands()
        count=0
        for path, orig in tqdm(split, desc=f"{outdir.name}"):
            tgt=j2y[orig]; y=cls_to_idx[tgt]
            seq=[]
            for frame in iter_sampled_frames(path):
                seq.append(feats_from_frame(frame, hp))
            if len(seq)<WIN: continue
            for win_arr in windows(seq):
                # Save one window per file (granularity good enough; change if needed)
                np.savez_compressed(outdir/f"w_{count}.npz", x=win_arr.astype(np.float32), y=np.int64(y))
                if collect_stats is not None:
                    collect_stats.append(win_arr.reshape(-1, FEATS))  # stack frames
                count+=1

    train_stats=[]
    process_split(train_files, out_root/'windows'/'train', collect_stats=train_stats)
    process_split(val_files,   out_root/'windows'/'val',   collect_stats=None)

    # normalizer from train frames
    if len(train_stats):
        A=np.concatenate(train_stats, axis=0)  # (N_frames,84)
        mean=A.mean(axis=0).astype(np.float32).tolist()
        std =(A.std(axis=0)+1e-6).astype(np.float32).tolist()
    else:
        mean=[0.0]*FEATS; std=[1.0]*FEATS

    (out_root/'labels.txt').write_text("\n".join(target_classes)+"\n")
    with open(out_root/'normalizer.json','w') as f:
        json.dump({'mean':mean,'std':std,'feature_order':'L(xy)*21 then R(xy)*21'}, f, indent=2)

    print("DONE →", out_root)
    print("Classes:", target_classes)

if __name__=='__main__':
    main()
