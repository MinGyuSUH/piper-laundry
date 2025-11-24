# detector/rgb_classifier.py
import torch
import torch.nn as nn
import torchvision.transforms as T
import torchvision
import numpy as np
import random
import os
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
from PIL import Image

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

class RGBOnlyDataset(Dataset):
    def __init__(self, base_dir, split='train', img_size=224):
        self.imgs, self.labels = [], []
        cloth_paths = []
        for cloth_dir in glob.glob(f"{base_dir}/cloth/*/rgb"):
            cloth_paths += glob.glob(f"{cloth_dir}/*.jpg") + glob.glob(f"{cloth_dir}/*.png")
        empty_paths = glob.glob(f"{base_dir}/empty/rgb/*.jpg") + glob.glob(f"{base_dir}/empty/rgb/*.png")

        cloth_paths.sort(); empty_paths.sort()
        random.seed(42)
        random.shuffle(cloth_paths)
        random.shuffle(empty_paths)
        cloth_cut = int(0.8 * len(cloth_paths))
        empty_cut = int(0.8 * len(empty_paths))

        if split == 'train':
            self.imgs = cloth_paths[:cloth_cut] + empty_paths[:empty_cut]
            self.labels = [1]*cloth_cut + [0]*empty_cut
        else:
            self.imgs = cloth_paths[cloth_cut:] + empty_paths[empty_cut:]
            self.labels = [1]*(len(cloth_paths)-cloth_cut) + [0]*(len(empty_paths)-empty_cut)

        self.transform = T.Compose([
            T.Resize((img_size, img_size)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225])
        ])

    def __len__(self): return len(self.imgs)

    def __getitem__(self, idx):
        img = Image.open(self.imgs[idx]).convert('RGB')
        return self.transform(img), self.labels[idx]

class RGBOnlyClassifier(nn.Module):
    def __init__(self):
        super().__init__()
        model = torchvision.models.resnet18(pretrained=True)
        model.fc = nn.Linear(512, 2)
        self.model = model

    def forward(self, x):
        return self.model(x)

def get_transforms():
    return T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

def evaluate(model, loader, device):
    model.eval(); correct = 0; total = 0
    for x, y in loader:
        x, y = x.to(device), y.to(device)
        pred = model(x).argmax(1)
        correct += (pred == y).sum().item(); total += y.size(0)
    return correct / total

def train_rgb_classifier(data_root, save_path='models/rgb_only_best.pth', epochs=30, bs=16, lr=1e-4):
    set_seed(42)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    ds_train = RGBOnlyDataset(data_root, 'train')
    ds_val   = RGBOnlyDataset(data_root, 'val')

    labels = ds_train.labels
    class_sample_count = np.bincount(labels)
    weight = 1. / torch.tensor(class_sample_count, dtype=torch.float)
    samples_weight = torch.tensor([weight[t] for t in labels])
    sampler = WeightedRandomSampler(samples_weight, len(samples_weight), replacement=True)

    dl_train = DataLoader(ds_train, batch_size=bs, sampler=sampler, num_workers=4)
    dl_val   = DataLoader(ds_val,   batch_size=bs, shuffle=False, num_workers=2)

    model = RGBOnlyClassifier().to(device)
    opt   = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = nn.CrossEntropyLoss()

    best_acc, best_epoch = 0, 0
    for epoch in range(1, epochs + 1):
        model.train()
        for x, y in dl_train:
            x, y = x.to(device), y.to(device)
            opt.zero_grad(); out = model(x)
            loss = criterion(out, y); loss.backward(); opt.step()
        acc = evaluate(model, dl_val, device)
        print(f"[Epoch {epoch:02d}] Val Acc: {acc:.4f}")
        if acc > best_acc:
            best_acc, best_epoch = acc, epoch
            torch.save(model.state_dict(), save_path)

    print(f"[Done] Best Acc = {best_acc:.4f} @ epoch {best_epoch}")
