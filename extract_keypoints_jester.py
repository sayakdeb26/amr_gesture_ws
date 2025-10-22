import os, cv2, numpy as np, pandas as pd
from collections import Counter
from tqdm import tqdm
import mediapipe as mp

DATA_DIR = os.environ["DATA_DIR"]
OUT_DIR  = os.environ["NPZ_DIR"]
os.makedirs(OUT_DIR, exist_ok=True)

T, LM = 30, 21
F = LM*3*2  # 126 features (x,y,z) for two hands

mp_hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)

def list_images(folder):
    try:
        return sorted([os.path.join(folder,f) for f in os.listdir(folder)
                       if f.lower().endswith((".jpg",".jpeg",".png"))])
    except FileNotFoundError:
        return []

def sample_T(items):
    if not items: return []
    idx = np.linspace(0, len(items)-1, num=T).round().astype(int).tolist()
    return [items[i] for i in idx]

def extract_xyzc(bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    res = mp_hands.process(rgb)
    hands=[]
    if res.multi_hand_landmarks:
        for hand in res.multi_hand_landmarks[:2]:
            xyz=[]
            for lm in hand.landmark: xyz.extend([float(lm.x), float(lm.y), float(lm.z)])
            hands.append(xyz)
    while len(hands)<2:
        hands.append([0.0]*(LM*3))
    return np.array(hands[0]+hands[1], dtype=np.float32)

def seq_from_frames(frames):
    sel = sample_T(frames)
    if not sel: return None
    seq = np.zeros((T,F), dtype=np.float32)
    for t,p in enumerate(sel):
        bgr = cv2.imread(p)
        if bgr is None: return None
        seq[t] = extract_xyzc(bgr)
    return seq

def read_csv_any(name):
    csvp = os.path.join(DATA_DIR, f"{name}.csv")
    if not os.path.isfile(csvp): return None
    df = pd.read_csv(csvp, header=None)
    # use first column as relpath, last column as label
    df = pd.DataFrame({"relpath": df.iloc[:,0].astype(str),
                       "label":   df.iloc[:,-1].astype(str)})
    return df

def iter_samples_from_csv(split_dir, df):
    base = os.path.join(DATA_DIR, split_dir)
    for rel, lab in df[["relpath","label"]].values:
        # try multiple resolutions of the relpath
        candidates = [
            os.path.join(base, rel),              # Train/<rel>
            os.path.join(base, lab, rel),         # Train/<label>/<rel>
            os.path.join(DATA_DIR, rel),          # CSV may already be relative to DATA_DIR
        ]
        yielded = False
        for cand in candidates:
            # sample is a folder containing frames
            frames = list_images(cand)
            if frames:
                yield lab, cand
                yielded = True
                break
            # else try one more depth if rel was a bare id under a class dir
            parts = rel.strip("/").split("/")
            if len(parts)==1:
                deeper = os.path.join(base, lab, parts[0])
                frames = list_images(deeper)
                if frames:
                    yield lab, deeper
                    yielded = True
                    break
        if not yielded:
            # skip silently if not found
            pass

def iter_samples_by_scan(split_dir):
    base = os.path.join(DATA_DIR, split_dir)
    if not os.path.isdir(base): return
    for cls in sorted(os.listdir(base)):
        cdir = os.path.join(base, cls)
        if not os.path.isdir(cdir): continue
        for sub in sorted(os.listdir(cdir)):
            sp = os.path.join(cdir, sub)
            fr = list_images(sp)
            if fr:
                yield cls, sp

def build_npz(split_dir, keep_labels, rows=None):
    X, y = [], []
    # choose iterator
    it = iter_samples_from_csv(split_dir, rows) if rows is not None else iter_samples_by_scan(split_dir)
    for cls, sp in tqdm(list(it), desc=split_dir):
        if cls not in keep_labels: continue
        seq = seq_from_frames(list_images(sp))
        if seq is None: continue
        X.append(seq); y.append(keep_labels.index(cls))
    if len(X)==0:
        X = np.zeros((0,T,F), np.float32); y = np.zeros((0,), np.int64)
    else:
        X = np.stack(X).astype(np.float32); y = np.array(y, np.int64)
    np.savez_compressed(os.path.join(OUT_DIR, f"{split_dir}.npz"), X=X, y=y)

def main():
    tr_df = read_csv_any("Train")
    va_df = read_csv_any("Validation")

    # count frequencies from CSV if available, else by scanning Train
    if tr_df is not None:
        counts = Counter(tr_df["label"])
    else:
        counts = Counter(cls for cls,_ in iter_samples_by_scan("Train"))
    top12 = [c for c,_ in counts.most_common(12)]

    with open(os.path.join(OUT_DIR,"labels.txt"), "w") as f:
        for c in top12: f.write(c+"\n")

    build_npz("Train", top12, rows=tr_df)
    build_npz("Validation", top12, rows=va_df)

if __name__ == "__main__":
    main()
