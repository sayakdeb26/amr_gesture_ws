import os, glob, argparse, numpy as np, torch, torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torch.optim.lr_scheduler as lr_scheduler
from torch.optim import AdamW

# -----------------------------
# Dataset
# -----------------------------
class AugmentedNpzWindows(Dataset):
    def __init__(self, root, augment=False):
        self.files = sorted(glob.glob(os.path.join(root, "*.npz")))
        self.augment = augment
        if not self.files:
            raise SystemExit(f"No .npz in {root}")
            
    def __len__(self):
        return len(self.files)
        
    def __getitem__(self, i):
        d = np.load(self.files[i])
        x = d["x"].astype("float32")   # (30,84)
        y = int(d["y"])
        
        if self.augment:
            x = self._augment_data(x)
            
        return torch.from_numpy(x), torch.tensor(y, dtype=torch.long)
    
    def _augment_data(self, x):
        # Add noise
        if np.random.random() > 0.5:
            noise = np.random.normal(0, 0.01, x.shape).astype("float32")
            x = x + noise
            
        # Random scaling
        if np.random.random() > 0.5:
            scale = np.random.uniform(0.9, 1.1)
            x = x * scale
            
        return x

# -----------------------------
# Model
# -----------------------------
class ImprovedLSTMClassifier(nn.Module):
    def __init__(self, in_dim=84, hidden=64, num_classes=12, num_layers=1, dropout=0.2):
        super().__init__()
        self.lstm = nn.LSTM(
            input_size=in_dim, 
            hidden_size=hidden, 
            num_layers=num_layers,
            batch_first=True, 
            bidirectional=True,
            dropout=0
        )
        self.head = nn.Sequential(
            nn.Linear(hidden * 2, 128),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(128, num_classes)
        )
    
    def forward(self, x):
        out, _ = self.lstm(x)
        out_mean = out.mean(dim=1)
        return self.head(out_mean)

# -----------------------------
# Train / Eval
# -----------------------------
def train_one_epoch(model, dl, opt, loss_fn, device, grad_clip=1.0):
    model.train()
    total = 0
    correct = 0
    total_loss = 0.0
    
    for x, y in dl:
        x = x.to(device, non_blocking=True)
        y = y.to(device, non_blocking=True)
        
        opt.zero_grad(set_to_none=True)
        logits = model(x)
        loss = loss_fn(logits, y)
        loss.backward()
        
        if grad_clip > 0:
            torch.nn.utils.clip_grad_norm_(model.parameters(), grad_clip)
            
        opt.step()
        
        total += y.numel()
        correct += (logits.argmax(1) == y).sum().item()
        total_loss += loss.item()
    
    avg_loss = total_loss / len(dl)
    accuracy = correct / total
    return accuracy, avg_loss

@torch.no_grad()
def eval_acc(model, dl, device):
    model.eval()
    total = 0
    correct = 0
    for x, y in dl:
        x = x.to(device, non_blocking=True)
        y = y.to(device, non_blocking=True)
        logits = model(x)
        total += y.numel()
        correct += (logits.argmax(1) == y).sum().item()
    return correct / total

# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--bs", type=int, default=64)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--num_layers", type=int, default=1)
    ap.add_argument("--dropout", type=float, default=0.2)
    ap.add_argument("--out_dir", default="/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls")
    ap.add_argument("--num_classes", type=int, default=12)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight_decay", type=float, default=1e-4)
    ap.add_argument("--grad_clip", type=float, default=1.0)
    ap.add_argument("--patience", type=int, default=8)
    args = ap.parse_args()

    # ---- device: prefer GPU in isolated_rosgpu ----
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # ---- your dataset paths ----
    train_dir = "/home/sayak/datasets/jester_npz/windows/train"
    val_dir   = "/home/sayak/datasets/jester_npz/windows/val"
    
    print(f"Training directory: {train_dir}")
    print(f"Validation directory: {val_dir}")
    print(f"Number of classes: {args.num_classes}")
    
    if not os.path.exists(train_dir):
        raise SystemExit(f"Training directory not found: {train_dir}")
    if not os.path.exists(val_dir):
        raise SystemExit(f"Validation directory not found: {val_dir}")
    
    train_ds = AugmentedNpzWindows(train_dir, augment=True)
    val_ds   = AugmentedNpzWindows(val_dir, augment=False)
    
    print(f"Training samples: {len(train_ds)}")
    print(f"Validation samples: {len(val_ds)}")
    
    # Quick label sanity check (first 100)
    all_labels = []
    for i in range(min(100, len(train_ds))):
        _, label = train_ds[i]
        all_labels.append(label.item())
    
    print(f"Label range in data: min={min(all_labels)}, max={max(all_labels)}")
    print(f"Unique labels (sample): {sorted(set(all_labels))}")
    
    train_dl = DataLoader(train_ds, batch_size=args.bs, shuffle=True,  num_workers=4, pin_memory=True)
    val_dl   = DataLoader(val_ds,   batch_size=args.bs, shuffle=False, num_workers=4, pin_memory=True)
    
    model = ImprovedLSTMClassifier(
        in_dim=84, 
        hidden=args.hidden, 
        num_classes=args.num_classes,
        num_layers=args.num_layers,
        dropout=args.dropout
    ).to(device)
    
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    opt = AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    loss_fn = nn.CrossEntropyLoss()
    
    scheduler = lr_scheduler.ReduceLROnPlateau(
        opt, mode="max", factor=0.5, patience=4
    )
    
    best_val_acc = 0.0
    patience_counter = 0
    
    os.makedirs(args.out_dir, exist_ok=True)
    
    # Save labels file (for ROS node later)
    labels_dst = os.path.join(args.out_dir, "labels.txt")
    with open(labels_dst, "w") as f:
        f.write("""IGNORE
NO_GESTURE
ROLL_BACK
ROLL_FWD
SWIPE_DOWN
SWIPE_LEFT
SWIPE_RIGHT
SWIPE_UP
THUMB_DOWN
THUMB_UP
ZOOM_IN
ZOOM_OUT""")
    print(f"Created labels file at: {labels_dst}")

    print("\nStarting training...")
    for ep in range(1, args.epochs + 1):
        train_acc, train_loss = train_one_epoch(model, train_dl, opt, loss_fn, device, args.grad_clip)
        val_acc = eval_acc(model, val_dl, device)
        
        old_lr = opt.param_groups[0]["lr"]
        scheduler.step(val_acc)
        new_lr = opt.param_groups[0]["lr"]
        
        if new_lr < old_lr:
            print(f"  -> LR reduced from {old_lr:.2e} to {new_lr:.2e}")
        
        print(f"[epoch {ep:3d}] train_acc={train_acc:.4f} train_loss={train_loss:.4f} "
              f"val_acc={val_acc:.4f} lr={new_lr:.2e}")
        
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            patience_counter = 0
            ckpt_path = os.path.join(args.out_dir, "best_model.pth")
            torch.save({
                "epoch": ep,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": opt.state_dict(),
                "val_acc": val_acc,
                "train_acc": train_acc,
            }, ckpt_path)
            print(f"  → New best model saved! (val_acc: {val_acc:.4f})")
        else:
            patience_counter += 1
            
        if patience_counter >= args.patience:
            print(f"Early stopping at epoch {ep}")
            break
    
    # -----------------------------
    # Export best model to ONNX (CPU)
    # -----------------------------
    print("\nLoading best model for export...")
    ckpt = torch.load(os.path.join(args.out_dir, "best_model.pth"), map_location="cpu")
    model_cpu = ImprovedLSTMClassifier(
        in_dim=84,
        hidden=args.hidden,
        num_classes=args.num_classes,
        num_layers=args.num_layers,
        dropout=args.dropout,
    )
    model_cpu.load_state_dict(ckpt["model_state_dict"])
    model_cpu.eval()
    print(f"Loaded best model from epoch {ckpt['epoch']} with val_acc={ckpt['val_acc']:.4f}")

    # 30x84 input
    dummy = torch.zeros(1, 30, 84, dtype=torch.float32)
    onnx_30 = os.path.join(args.out_dir, "model_30x84.onnx")
    torch.onnx.export(
        model_cpu, dummy, onnx_30,
        input_names=["x"], output_names=["logits"],
        dynamic_axes={"x": {0: "batch"}, "logits": {0: "batch"}},
        opset_version=17,
    )
    print("Saved", onnx_30)

    # Flattened 2520 input
    class FlatWrap(nn.Module):
        def __init__(self, core):
            super().__init__()
            self.core = core
        def forward(self, z):
            B = z.shape[0]
            x = z.view(B, 30, 84)
            return self.core(x)

    flat = FlatWrap(model_cpu)
    dummy_flat = torch.zeros(1, 30 * 84, dtype=torch.float32)
    onnx_2520 = os.path.join(args.out_dir, "model_2520.onnx")
    torch.onnx.export(
        flat, dummy_flat, onnx_2520,
        input_names=["x"], output_names=["logits"],
        dynamic_axes={"x": {0: "batch"}, "logits": {0: "batch"}},
        opset_version=17,
    )
    print("Saved", onnx_2520)
    
    print(f"\nTraining completed! Best validation accuracy: {best_val_acc:.4f}")
    print(f"All files saved to: {args.out_dir}")

if __name__ == "__main__":
    main()

