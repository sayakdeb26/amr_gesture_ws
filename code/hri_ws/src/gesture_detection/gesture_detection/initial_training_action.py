import os
import json
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, ConcatDataset
from avalanche.benchmarks import nc_benchmark
from avalanche.training import Naive
from avalanche.evaluation.metrics import accuracy_metrics, loss_metrics
from avalanche.logging import InteractiveLogger
from avalanche.training.plugins import EvaluationPlugin

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

# Import your action definition
from gesture_detection_interface.action import Retrain

CLASSES_FILE = "src/gesture_detection/resource/classes.json"
DATA_DIR = "src/gesture_detection/gesture_detection/data"
MODEL_FILE = "src/gesture_detection/gesture_detection/model/gesture_model_cnn_lstm.pth"


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
        x = x.transpose(1, 2)  # (batch, features, seq_len)
        x = torch.relu(self.bn1(self.conv1(x)))
        x = torch.relu(self.bn2(self.conv2(x)))
        x = x.transpose(1, 2)  # back to (batch, seq_len, channels)
        _, (hn, _) = self.lstm(x)
        out = self.fc(hn[-1])
        return out


def load_classes(filename=CLASSES_FILE):
    if not os.path.exists(filename):
        raise FileNotFoundError(f"{filename} not found. Please create it first.")
    with open(filename, "r") as f:
        return json.load(f)


def load_npz_dataset(path):
    data = np.load(path)
    X = torch.tensor(data["X"], dtype=torch.float32)
    y = torch.tensor(data["y"], dtype=torch.long)
    return TensorDataset(X, y)


class TrainInitialModelActionServer(Node):
    def __init__(self):
        super().__init__('train_initial_model_action_server')

        self._action_server = ActionServer(
            self,
            Retrain,
            'train_gesture_model',
            execute_callback=self.execute_callback
        )
        self.get_logger().info("TrainGestureModel Action Server started")

    async def execute_callback(self, goal_handle):
        trainingdata = goal_handle.request.trainingdata
        self.get_logger().info(f'Received training goal with trainingdata={trainingdata}')

        feedback_msg = Retrain.Feedback()
        result = Retrain.Result()

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        if not os.path.exists(DATA_DIR):
            self.get_logger().error(f"Data directory '{DATA_DIR}' not found.")
            result.modelpath = ''
            goal_handle.abort()
            return result

        gesture_files = [os.path.join(DATA_DIR, f) for f in os.listdir(DATA_DIR) if f.endswith(".npz")]
        if not gesture_files:
            self.get_logger().error(f"No .npz gesture files found in '{DATA_DIR}'.")
            result.modelpath = ''
            goal_handle.abort()
            return result

        datasets = [load_npz_dataset(path) for path in gesture_files]
        full_dataset = ConcatDataset(datasets)

        try:
            classes = load_classes()
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            result.modelpath = ''
            goal_handle.abort()
            return result

        num_classes = len(classes)

        seq_len = full_dataset[0][0].shape[0]
        input_dim = full_dataset[0][0].shape[1]
        if input_dim != 63:
            self.get_logger().error(f"Expected input_dim=63, but got {input_dim}")
            result.modelpath = ''
            goal_handle.abort()
            return result

        benchmark = nc_benchmark(
            [full_dataset],
            [full_dataset],
            n_experiences=1,
            task_labels=False,
            shuffle=False
        )

        model = CNN_LSTM_Classifier(
            seq_len=seq_len,
            input_dim=input_dim,
            hidden_dim=64,
            output_dim=num_classes
        ).to(device)

        optimizer = optim.Adam(model.parameters(), lr=1e-3)
        criterion = nn.CrossEntropyLoss()

        interactive_logger = InteractiveLogger()
        eval_plugin = EvaluationPlugin(
            accuracy_metrics(experience=True, stream=True),
            loss_metrics(experience=True, stream=True),
            loggers=[interactive_logger]
        )

        strategy = Naive(
            model=model,
            optimizer=optimizer,
            criterion=criterion,
            train_mb_size=8,
            train_epochs=10,
            eval_mb_size=16,
            device=device,
            evaluator=eval_plugin
        )

        self.get_logger().info("Starting training on all gestures...")

        total_epochs = 10
        current_epoch = 0

        for experience in benchmark.train_stream:
            for epoch in range(total_epochs):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Training canceled by client request.')
                    goal_handle.canceled()
                    result.modelpath = ''
                    return result

                results = strategy.train(experience)
                current_epoch += 1

                # Publish feedback (progress percentage)
                progress_percent = int((current_epoch / total_epochs) * 100)
                feedback_msg.progress = progress_percent
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Training progress: {progress_percent}%")

        self.get_logger().info("Evaluating trained model...")
        for experience in benchmark.test_stream:
            eval_results = strategy.eval(experience)
            self.get_logger().info(f"Evaluation results: {eval_results}")

        torch.save(model.state_dict(), MODEL_FILE)
        self.get_logger().info(f"Model saved to {MODEL_FILE}")

        result.modelpath = MODEL_FILE
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = TrainInitialModelActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
