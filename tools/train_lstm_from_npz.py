import os, glob, argparse, numpy as np, torch, torch.nn as nn
from torch.utils.data import Dataset, DataLoader

class NpzWindows(Dataset):
    def __init__(self, root):
        self.files = sorted(glob.glob(os.path.join(root, "*.npz")))
        if not self.files: raise SystemExit(f"No .npz in {root}")
    def __len__(self): return len(self.files)
    def __getitem__(self, i):
        d = np.load(self.files[i])
        x = d["x"].astype("float32")   # (30,84)
        y = int(d["y"])
        return torch.from_numpy(x), torch.tensor(y, dtype=torch.long)

class LSTMClassifier(nn.Module):
    def __init__(self, in_dim=84, hidden=64, num_classes=5):
        super().__init__()
        self.lstm = nn.LSTM(input_size=in_dim, hidden_size=hidden, num_layers=1,
                            batch_first=True, bidirectional=False)
        self.head = nn.Linear(hidden, num_classes)
    def forward(self, x):          # x: (B,30,84)
        out,_ = self.lstm(x)       # (B,30,H)
        last = out[:,-1,:]         # (B,H) use last timestep
        return self.head(last)     # (B,C)

def train_one_epoch(model, dl, opt, loss_fn, device):
    model.train(); total=0; correct=0
    for x,y in dl:
        x=x.to(device); y=y.to(device)
        opt.zero_grad()
        logits = model(x)
        loss = loss_fn(logits, y)
        loss.backward(); opt.step()
        total += y.numel(); correct += (logits.argmax(1)==y).sum().item()
    return correct/total

@torch.no_grad()
def eval_acc(model, dl, device):
    model.eval(); total=0; correct=0
    for x,y in dl:
        x=x.to(device); y=y.to(device)
        logits = model(x)
        total += y.numel(); correct += (logits.argmax(1)==y).sum().item()
    return correct/total

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--prepared_root", required=True)
    ap.add_argument("--epochs", type=int, default=5)
    ap.add_argument("--bs", type=int, default=64)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--num_classes", type=int, default=5)
    args=ap.parse_args()

    train_dir = os.path.join(args.prepared_root,"windows","train")
    val_dir   = os.path.join(args.prepared_root,"windows","val")
    train_ds, val_ds = NpzWindows(train_dir), NpzWindows(val_dir)
    train_dl = DataLoader(train_ds, batch_size=args.bs, shuffle=True, drop_last=False)
    val_dl   = DataLoader(val_ds,   batch_size=args.bs, shuffle=False, drop_last=False)

    device = torch.device("cpu")
    model = LSTMClassifier(in_dim=84, hidden=args.hidden, num_classes=args.num_classes).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.CrossEntropyLoss()

    for ep in range(1, args.epochs+1):
        train_acc = train_one_epoch(model, train_dl, opt, loss_fn, device)
        val_acc   = eval_acc(model, val_dl, device)
        print(f"[epoch {ep}] train_acc={train_acc:.3f} val_acc={val_acc:.3f}")

    os.makedirs(args.out_dir, exist_ok=True)

    # Export ONNX with input shape (1,30,84)
    dummy = torch.zeros(1,30,84, dtype=torch.float32)
    onnx_30 = os.path.join(args.out_dir, "model_30x84.onnx")
    torch.onnx.export(model, dummy, onnx_30, input_names=["x"], output_names=["logits"], opset_version=18, dynamo=False, do_constant_folding=True,
                      dynamic_axes={"x":{0:"batch"}, "logits":{0:"batch"}})
    print("Saved", onnx_30)

    # Export a flattened wrapper (1,2520) → (1,30,84) → model
    class FlatWrap(nn.Module):
        def __init__(self, core): super().__init__(); self.core=core
        def forward(self, z):        # z: (B,2520)
            B = z.shape[0]
            x = z.view(B,30,84)
            return self.core(x)
    flat = FlatWrap(model)
    dummy_flat = torch.zeros(1,30*84, dtype=torch.float32)
    onnx_2520 = os.path.join(args.out_dir, "model_2520.onnx")
    torch.onnx.export(flat, dummy_flat, onnx_2520, input_names=["x"], output_names=["logits"], opset_version=18, dynamo=False, do_constant_folding=True,
                      dynamic_axes={"x":{0:"batch"}, "logits":{0:"batch"}})
    print("Saved", onnx_2520)

if __name__=="__main__":
    main()
