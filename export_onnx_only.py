import os, json, numpy as np, torch, torch.nn as nn
NPZ_DIR   = os.environ.get("NPZ_DIR", os.path.expanduser("~/datasets/jester_npz"))
MODEL_DIR = os.environ.get("MODEL_DIR", os.path.expanduser("~/amr_gesture_ws/models/lstm/jester20b_12cls"))
os.makedirs(MODEL_DIR, exist_ok=True)

# load shapes + normalizer (recompute if missing)
train = np.load(os.path.join(NPZ_DIR,"Train.npz"))
Xtr = train["X"].astype("float32")
T, F = Xtr.shape[1], Xtr.shape[2]

norm_path = os.path.join(MODEL_DIR,"normalizer.json")
if os.path.isfile(norm_path):
    with open(norm_path) as f: j=json.load(f)
    mu = np.array(j["mu"], dtype=np.float32); sigma = np.array(j["sigma"], dtype=np.float32)
else:
    mu = Xtr.reshape(-1,F).mean(axis=0); sigma = Xtr.reshape(-1,F).std(axis=0); sigma[sigma==0]=1.0
    with open(norm_path,"w") as f: json.dump({"mu":mu.tolist(),"sigma":sigma.tolist()}, f)

class LstmClassifier(nn.Module):
    def __init__(self, f, c, hidden=128, layers=2, dr=0.2):
        super().__init__()
        self.lstm = nn.LSTM(f, hidden, layers, batch_first=True, dropout=dr, bidirectional=True)
        self.head = nn.Sequential(nn.Linear(hidden*2,256), nn.ReLU(), nn.Dropout(dr), nn.Linear(256,c))
    def forward(self, x):
        o,_ = self.lstm(x)
        return self.head(o[:,-1,:])

labels_path_npz = os.path.join(NPZ_DIR,"labels.txt")
labels_path_out = os.path.join(MODEL_DIR,"labels.txt")
if os.path.isfile(labels_path_npz):
    import shutil; shutil.copy(labels_path_npz, labels_path_out)

# infer num classes from labels
if os.path.isfile(labels_path_out):
    with open(labels_path_out) as f: C = sum(1 for _ in f if _.strip())
else:
    C = 12

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = LstmClassifier(F, C).to(device)

# load best.pt if present
best = os.path.join(MODEL_DIR,"best.pt")
if os.path.isfile(best):
    model.load_state_dict(torch.load(best, map_location=device))
model.eval()

dummy = torch.zeros(1, T, F, dtype=torch.float32, device=device)
torch.onnx.export(model, dummy, os.path.join(MODEL_DIR,"model.onnx"),
                  input_names=["x"], output_names=["logits"], opset_version=17,
                  dynamic_axes={"x": {0:"batch", 1:"time"}, "logits": {0:"batch"}})
print("Exported:", os.path.join(MODEL_DIR,"model.onnx"))
