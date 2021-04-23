import os
import pdb
import traceback
import sys
import time
import torch
import shutil
import numpy as np
from torch import nn
from torch import optim
from datetime import datetime
from torch.nn import functional as F
from torch.utils.tensorboard import SummaryWriter


class MLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(5, 16),
            # nn.Dropout(0.4),
            nn.ReLU(True),
            nn.Linear(16, 16),
            nn.Dropout(0.4),
            nn.ReLU(True),
            nn.Linear(16, 2)
        )

    def forward(self, x):
        return self.layers(x)


def focalLoss(p, label, gamma=2.0):
    pos_term = label * ((1 - p) ** gamma)
    neg_term = (1 - label) * (p ** gamma)

    # Term involving the log and ReLU
    log_weight = pos_term + neg_term
    log_term = torch.log1p(torch.exp(-torch.abs(p)))
    log_term += F.relu(-p)
    log_term *= log_weight

    # Combine all the terms into the loss
    loss = neg_term * p + log_term
    return loss.sum()

def train(del_dir):
    epoches = 300
    batch_size = 512

    logdir = 'logs/'
    if os.path.exists(logdir) and del_dir:
        shutil.rmtree(logdir)
    
    time_stamp = "{0:%Y-%m-%d/%H-%M-%S}-batch{1}-epoch{2}/".format(datetime.now(), batch_size, epoches)
    writer = SummaryWriter(log_dir = logdir+time_stamp)

    model = MLP()
    optimizer = optim.Adam(model.parameters())

    data_raw = np.loadtxt("./data/adata.csv", np.float32, delimiter=',')
    # np.random.shuffle(data_raw)

    train_num = data_raw.shape[0] * 15 // 16 
    x_train = torch.from_numpy(data_raw[:train_num, :5])
    y_train = torch.from_numpy(data_raw[:train_num, 5:]).long()
    x_valid = torch.from_numpy(data_raw[train_num:, :5])
    y_valid = torch.from_numpy(data_raw[train_num:, 5:]).long()

    batch_num = x_train.shape[0] // batch_size
    loss_func = focalLoss

    for epoch in range(epoches):
        model.train()
        loss = 0
        fault_cnt = 0
        for batch_idx in range(batch_num):
            x = x_train[batch_size * batch_idx: batch_size * (batch_idx+1)]
            y = y_train[batch_size * batch_idx: batch_size * (batch_idx+1)]
            y_pred = model(x)
            # print("Y pred shape:", y_pred.shape, ", x shape:", x.shape)
            loss_ = loss_func(y_pred, y)
            optimizer.zero_grad()
            loss_.backward()
            optimizer.step()
            loss += loss_.item() / batch_size
            pred, _ = torch.max(torch.abs(y_pred - y), dim = 1)
            fault_cnt += pred.clamp(0, 1).round().sum().item()
        loss /= batch_num
        acc = 1 - fault_cnt / x_train.shape[0]

        model.eval()
        with torch.no_grad():
            y_pred = model(x_valid)
            valid_loss = loss_func(y_pred, y_valid) / x_valid.shape[0]

            pred, _ = torch.max(torch.abs(y_pred - y_valid), dim = 1)
            valid_acc = 1 - pred.clamp(0, 1).round().sum().item() / y_valid.shape[0]

            # pred = y_pred.argmax(dim=1, keepdim=True)
            # valid_acc = pred.eq(y_valid.view_as(pred)).sum().item() / x_valid.shape[0]
        writer.add_scalar('Loss/train', loss, epoch)
        writer.add_scalar('Accuracy/train', acc, epoch)
        writer.add_scalar('Loss/valid', valid_loss, epoch)
        writer.add_scalar('Accuracy/valid', valid_acc, epoch)
        if (epoch+1) % 1 == 0:
            print('Train Epoch: {} / {}\tLoss: {:.6f}\tAcc: {:.6f}\tValidLoss: {:.6f}\tValidAcc: {:.6f}'.format(
                epoch, epoches, loss, acc, valid_loss, valid_acc))
    torch.save(model.state_dict(), "mlp.pt")
    
    writer.close()

def predict():
    model = MLP()
    stat_dict = torch.load("mlp.pt")
    model.load_state_dict(stat_dict)

    model.eval()
    example = torch.zeros((1, 5), dtype = torch.float32)
    traced_script = torch.jit.trace(model, example)
    traced_script.save("traced_model.pt")

def main():
    if len(sys.argv) < 2:
        print("Usage: python ./MLP.py <flag>(flag = 0 means training, flag = 1 means using trained model)")
    do_train = int(sys.argv[1])
    if do_train == 0:
        del_dir = 0
        if len(sys.argv) > 2:
            del_dir = int(sys.argv[2])
        train(del_dir)
    else:
        predict()


if __name__ == '__main__':
    try:
        main()
    except Exception as err:
        last_traceback = sys.exc_info()[2]
        traceback.print_tb(last_traceback)
        print(err)
        pdb.post_mortem(last_traceback)
