import os, json, numpy as np, torch, torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from tqdm import tqdm

NPZ_DIR   = os.environ["NPZ_DIR"]
MODEL_DIR = os.environ["MODEL_DIR"]
os.makedirs(MODEL_DIR, exist_ok=True)

train = np.load(os.path.join(NPZ_DIR,"Train.npz"))
val   = np.load(os.path.join(NPZ_DIR,"Validation.npz"))
Xtr,ytr = train["X"].astype("float32"), train["y"].astype("int64")
Xva,yva = val["X"].astype("float32"),   val["y"].astype("int64")
T,F = Xtr.shape[1], Xtr.shape[2]
C = int(max(ytr.max(), yva.max())+1) if Xtr.shape[0] else 12

# per-feature normalizer
mu = Xtr.reshape(-1,F).mean(axis=0)
sigma = Xtr.reshape(-1,F).std(axis=0); sigma[sigma==0]=1.0
def norm(x): return (x-mu)/sigma
Xtr, Xva = norm(Xtr), norm(Xva)
with open(os.path.join(MODEL_DIR,"normalizer.json"),"w") as f:
    json.dump({"mu":mu.tolist(),"sigma":sigma.tolist()}, f)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class LstmClassifier(nn.Module):
    def __init__(self, f, c, hidden=128, layers=2, dropout=0.2):
        super().__init__()
        self.lstm = nn.LSTM(input_size=f, hidden_size=hidden, num_layers=layers,
                            batch_first=True, dropout=dropout, bidirectional=True)
        self.head = nn.Sequential(nn.Linear(hidden*2,256), nn.ReLU(), nn.Dropout(0.2), nn.Linear(256,c))
    def forward(self, x):
        o,_ = self.lstm(x)
        return self.head(o[:, -1, :])

def mkloader(X,y,b=64,sh=True):
    return DataLoader(TensorDataset(torch.from_numpy(X), torch.from_numpy(y)),
                      batch_size=b, shuffle=sh, num_workers=2, pin_memory=True)

tr, va = mkloader(Xtr,ytr), mkloader(Xva,yva,sh=False)
model = LstmClassifier(F, C).to(device)
opt   = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=1e-3)
crit  = nn.CrossEntropyLoss()

best=0.0
for epoch in range(15):
    model.train()
    for xb,yb in tqdm(tr, desc=f"epoch {epoch+1}/15"):
        xb,yb = xb.to(device), yb.to(device)
        opt.zero_grad(); loss=crit(model(xb), yb); loss.backward(); opt.step()
    model.eval(); corr=tot=0
    with torch.no_grad():
        for xb,yb in va:
            xb,yb = xb.to(device), yb.to(device)
            pred = model(xb).argmax(1)
            corr += (pred==yb).sum().item(); tot += yb.numel()
    acc = corr/max(1,tot); 
    if acc>best: best=acc; torch.save(model.state_dict(), os.path.join(MODEL_DIR,"best.pt"))
    print(f"val_acc={acc:.3f} best={best:.3f}")

# save labels
import shutil
shutil.copy(os.path.join(NPZ_DIR,"labels.txt"), os.path.join(MODEL_DIR,"labels.txt"))

# export ONNX
model.load_state_dict(torch.load(os.path.join(MODEL_DIR,"best.pt"), map_location=device))
model.eval()
dummy = torch.zeros(1, T, F, dtype=torch.float32, device=device)
torch.onnx.export(model, dummy, os.path.join(MODEL_DIR,"model.onnx"),
                  input_names=["x"], output_names=["logits"], opset_version=17,
                  dynamic_axes={"x": {0: "batch", 1: "time"}, "logits": {0: "batch"}})
print("Exported:", os.path.join(MODEL_DIR,"model.onnx"))
