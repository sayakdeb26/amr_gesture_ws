import os
import json
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from torch.utils.data import TensorDataset, ConcatDataset
from gesture_detection.action import Retrain

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from avalanche.benchmarks import nc_benchmark
from avalanche.training import EWC, Naive
from avalanche.evaluation.metrics import accuracy_metrics, loss_metrics
from avalanche.logging import InteractiveLogger
from avalanche.training.plugins import EvaluationPlugin

CLASSES_FILE = "classes.json"
MODEL_FILE = "gesture_model_cnn_lstm.pth"
HIDDEN_DIM = 64
NEW_DATA_DIR = "data_new"

class CNN_LSTM_Classifier(nn.Module):
    def __init__(self, seq_len=30, input_dim=63, hidden_dim=64, output_dim=10):
        super().__init__()
        self.seq_len = seq_len

        self.conv1 = nn.Conv1d(in_channels=input_dim, out_channels=64, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm1d(64)
        self.conv2 = nn.Conv1d(in_channels=64, out_channels=128, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm1d(128)

        self.lstm = nn.LSTM(
            input_size=128,
            hidden_size=hidden_dim,
            num_layers=2,
            batch_first=True,
            dropout=0.3
        )

        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        x = x.transpose(1, 2)  # (batch, features, seq_len) for Conv1d
        x = torch.relu(self.bn1(self.conv1(x)))
        x = torch.relu(self.bn2(self.conv2(x)))
        x = x.transpose(1, 2)  # back to (batch, seq_len, channels) for LSTM
        _, (hn, _) = self.lstm(x)
        out = self.fc(hn[-1])
        return out

def load_class_list():
    if os.path.exists(CLASSES_FILE):
        with open(CLASSES_FILE, "r") as f:
            return json.load(f)
    else:
        return []

def save_class_list(classes):
    with open(CLASSES_FILE, "w") as f:
        json.dump(classes, f)

def load_npz_dataset(path):
    data = np.load(path)
    X = torch.tensor(data["X"], dtype=torch.float32)
    y = torch.tensor(data["y"], dtype=torch.long)
    return TensorDataset(X, y)

def load_all_datasets(folder):
    datasets = []
    labels_found = set()
    for file in sorted(os.listdir(folder)):
        if file.endswith(".npz"):
            ds = load_npz_dataset(os.path.join(folder, file))
            datasets.append(ds)
            y_vals = np.load(os.path.join(folder, file))["y"]
            labels_found.update(y_vals.tolist())
    return datasets, sorted(labels_found)

def expand_output_layer_from_old(old_model, old_dim, new_dim, device):
    if new_dim == old_dim:
        return old_model
    in_f = old_model.fc.in_features
    new_fc = nn.Linear(in_f, new_dim).to(device)
    with torch.no_grad():
        new_fc.weight[:old_dim].copy_(old_model.fc.weight[:old_dim])
        new_fc.bias[:old_dim].copy_(old_model.fc.bias[:old_dim])
    old_model.fc = new_fc
    return old_model

class TrainModelActionServer(Node):
    def __init__(self):
        super().__init__('train_model_action_server')

        self._action_server = ActionServer(
            self,
            Retrain,
            'train_model',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Training goal received')

        feedback_msg = Retrain.Feedback()
        result = Retrain.Result()

        try:
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

            if not os.path.exists(NEW_DATA_DIR) or not os.listdir(NEW_DATA_DIR):
                raise FileNotFoundError(f"No new gesture data found in '{NEW_DATA_DIR}'")

            feedback_msg.status = "Loading new datasets"
            goal_handle.publish_feedback(feedback_msg)

            new_datasets, new_labels = load_all_datasets(NEW_DATA_DIR)

            feedback_msg.status = "Loading existing classes"
            goal_handle.publish_feedback(feedback_msg)

            classes = load_class_list()
            all_labels = sorted(set(classes) | set(new_labels))
            if classes != all_labels:
                self.get_logger().info(f"Updating classes.json: {classes} -> {all_labels}")
                classes = all_labels
                save_class_list(classes)
            num_classes = len(classes)

            from avalanche.benchmarks import nc_benchmark
            from avalanche.training import EWC, Naive
            from avalanche.evaluation.metrics import accuracy_metrics, loss_metrics
            from avalanche.logging import InteractiveLogger
            from avalanche.training.plugins import EvaluationPlugin

            new_dataset_full = ConcatDataset(new_datasets)
            benchmark = nc_benchmark(
                [new_dataset_full],
                [new_dataset_full],
                n_experiences=1,
                task_labels=False,
                shuffle=False
            )

            input_dim = new_dataset_full[0][0].shape[1]
            seq_len = new_dataset_full[0][0].shape[0]
            assert input_dim == 63, f"Expected input_dim=63, got {input_dim}"

            if not os.path.exists(MODEL_FILE):
                raise FileNotFoundError(f"No existing model found at {MODEL_FILE} to fine-tune.")

            feedback_msg.status = "Loading existing model"
            goal_handle.publish_feedback(feedback_msg)

            state = torch.load(MODEL_FILE, map_location=device)
            prev_out_dim = state['fc.weight'].shape[0]
            model = CNN_LSTM_Classifier(seq_len=seq_len, input_dim=input_dim, hidden_dim=HIDDEN_DIM, output_dim=prev_out_dim).to(device)
            model.load_state_dict(state)

            if prev_out_dim != num_classes:
                feedback_msg.status = f"Expanding output layer: {prev_out_dim} -> {num_classes}"
                goal_handle.publish_feedback(feedback_msg)
                model = expand_output_layer_from_old(model, prev_out_dim, num_classes, device)

            optimizer = optim.Adam(model.parameters(), lr=1e-3)
            criterion = nn.CrossEntropyLoss()

            interactive_logger = InteractiveLogger()
            eval_plugin = EvaluationPlugin(
                accuracy_metrics(experience=True, stream=True),
                loss_metrics(experience=True, stream=True),
                loggers=[interactive_logger]
            )

            # You can toggle EWC usage here or pass as param
            use_ewc = True
            train_epochs = 8
            train_mb_size = 8
            eval_mb_size = 16
            ewc_lambda = 0.4

            if use_ewc:
                feedback_msg.status = "Starting EWC training"
                goal_handle.publish_feedback(feedback_msg)

                strategy = EWC(
                    model=model,
                    optimizer=optimizer,
                    criterion=criterion,
                    ewc_lambda=ewc_lambda,
                    train_mb_size=train_mb_size,
                    train_epochs=train_epochs,
                    eval_mb_size=eval_mb_size,
                    device=device,
                )
            else:
                feedback_msg.status = "Starting Naive training"
                goal_handle.publish_feedback(feedback_msg)

                strategy = Naive(
                    model=model,
                    optimizer=optimizer,
                    criterion=criterion,
                    train_mb_size=train_mb_size,
                    train_epochs=train_epochs,
                    eval_mb_size=eval_mb_size,
                    device=device,
                )

            for experience in benchmark.train_stream:
                feedback_msg.status = f"Training on {len(experience.dataset)} samples"
                goal_handle.publish_feedback(feedback_msg)
                strategy.train(experience)

            for experience in benchmark.test_stream:
                strategy.eval(experience)

            torch.save(model.state_dict(), MODEL_FILE)
            self.get_logger().info(f"Model saved to {MODEL_FILE}")

            result.success = True
            result.message = f"Training complete. Model saved to {MODEL_FILE}"
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Training failed: {e}")
            result.success = False
            result.message = f"Training failed: {e}"
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)
    action_server = TrainModelActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
