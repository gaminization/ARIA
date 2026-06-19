#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
Neural IK Solver for the ARIA 5-DoF Arm
Small MLP trained to map pose → joint angles for THIS arm.

Architecture: 7 inputs (xyz + quaternion xyzw)
              → 256 hidden (ReLU)
              → 128 hidden (ReLU)
              → 5 outputs (joint angles in radians)

Training: FK-based data generation → supervised learning
Inference: <1ms on GPU (RTX 5060)
═══════════════════════════════════════════════════════════════
"""
import math
import os
import time
from typing import Optional

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

from arm_ik.ik_solvers.aria_analytical_ik import (
    IKResult, SolverConfig, JOINT_LIMITS, forward_kinematics
)


# ═══════════════════════════════════════════════════════════════
# Neural Network Architecture
# ═══════════════════════════════════════════════════════════════
if TORCH_AVAILABLE:
    class IKNet(nn.Module):
        """
        MLP for inverse kinematics.

        Input:  7D — [x, y, z, qx, qy, qz, qw] (normalized)
        Output: 5D — [θ1, θ2, θ3, θ4, θ5] (normalized to [-1, 1])

        Hidden layers: 256 → 128 with ReLU activation.
        Output: tanh to bound outputs, then scale to joint limits.
        """

        def __init__(self):
            super().__init__()
            self.network = nn.Sequential(
                nn.Linear(7, 256),
                nn.ReLU(),
                nn.BatchNorm1d(256),
                nn.Dropout(0.1),
                nn.Linear(256, 128),
                nn.ReLU(),
                nn.BatchNorm1d(128),
                nn.Dropout(0.1),
                nn.Linear(128, 64),
                nn.ReLU(),
                nn.Linear(64, 5),
                nn.Tanh(),  # Output in [-1, 1]
            )

            # Joint limit ranges for denormalization
            self.register_buffer(
                'joint_mins',
                torch.tensor(JOINT_LIMITS[:, 0], dtype=torch.float32)
            )
            self.register_buffer(
                'joint_maxs',
                torch.tensor(JOINT_LIMITS[:, 1], dtype=torch.float32)
            )

        def forward(self, x):
            """Forward pass: normalized pose → normalized joints."""
            return self.network(x)

        def denormalize_joints(self, normalized_output):
            """Convert tanh output [-1, 1] → joint angle range."""
            # Map [-1, 1] → [min, max] for each joint
            return (normalized_output + 1.0) / 2.0 * (
                self.joint_maxs - self.joint_mins
            ) + self.joint_mins

        def normalize_joints(self, joint_angles):
            """Convert joint angles → [-1, 1] for training targets."""
            return 2.0 * (joint_angles - self.joint_mins) / (
                self.joint_maxs - self.joint_mins
            ) - 1.0


# ═══════════════════════════════════════════════════════════════
# Training Data Generation
# ═══════════════════════════════════════════════════════════════
def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3×3 rotation matrix to quaternion [qx, qy, qz, qw]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)  # Normalize


def generate_training_data(n_samples: int = 50000,
                           save_path: str = None) -> tuple:
    """
    Generate training data by sampling valid joint configs → computing FK.

    For each sample:
      1. Random joint angles within limits
      2. Compute FK → get end-effector pose
      3. Store (pose, joints) pair

    Args:
        n_samples: number of training samples
        save_path: path to save .npz file (optional)

    Returns:
        (poses, joints) — numpy arrays of shape (N, 7) and (N, 5)
    """
    print(f"Generating {n_samples} training samples...")

    poses = np.zeros((n_samples, 7))    # [x, y, z, qx, qy, qz, qw]
    joints = np.zeros((n_samples, 5))   # [θ1..θ5]

    rng = np.random.default_rng(42)  # Reproducible

    for i in range(n_samples):
        # Sample random valid joint configuration
        q = np.array([
            rng.uniform(JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1])
            for j in range(5)
        ])

        # Compute FK
        T = forward_kinematics(q)
        pos = T[:3, 3]
        rot = T[:3, :3]
        quat = _rotation_matrix_to_quaternion(rot)

        poses[i, :3] = pos
        poses[i, 3:] = quat
        joints[i] = q

        if (i + 1) % 10000 == 0:
            print(f"  Generated {i + 1}/{n_samples} samples")

    if save_path:
        np.savez(save_path, poses=poses, joints=joints)
        print(f"Saved training data to {save_path}")

    return poses, joints


def train_neural_ik(data_path: str = None,
                    model_path: str = None,
                    epochs: int = 100,
                    batch_size: int = 256,
                    lr: float = 1e-3) -> Optional['IKNet']:
    """
    Train the neural IK model.

    Args:
        data_path: path to training data .npz
        model_path: path to save trained model .pt
        epochs: number of training epochs
        batch_size: training batch size
        lr: learning rate

    Returns:
        Trained IKNet model
    """
    if not TORCH_AVAILABLE:
        print("PyTorch not available, cannot train neural IK")
        return None

    # Generate or load data
    if data_path and os.path.exists(data_path):
        data = np.load(data_path)
        poses, joints = data['poses'], data['joints']
    else:
        poses, joints = generate_training_data(50000, data_path)

    # Normalize inputs: position to [-1, 1] range
    pos_mean = poses[:, :3].mean(axis=0)
    pos_std = poses[:, :3].std(axis=0) + 1e-8

    poses_norm = poses.copy()
    poses_norm[:, :3] = (poses[:, :3] - pos_mean) / pos_std
    # Quaternions are already normalized

    # Split train/val (90/10)
    n = len(poses)
    n_train = int(0.9 * n)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Training on: {device}")

    model = IKNet().to(device)

    # Normalize joint targets to [-1, 1]
    joints_tensor = torch.tensor(joints, dtype=torch.float32).to(device)
    joints_norm = model.normalize_joints(joints_tensor)

    poses_tensor = torch.tensor(poses_norm, dtype=torch.float32).to(device)

    train_dataset = TensorDataset(
        poses_tensor[:n_train], joints_norm[:n_train]
    )
    val_dataset = TensorDataset(
        poses_tensor[n_train:], joints_norm[n_train:]
    )

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size)

    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=5, factor=0.5
    )
    criterion = nn.MSELoss()

    best_val_loss = float('inf')

    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        for batch_poses, batch_joints in train_loader:
            optimizer.zero_grad()
            pred = model(batch_poses)
            loss = criterion(pred, batch_joints)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(batch_poses)

        train_loss /= n_train

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for batch_poses, batch_joints in val_loader:
                pred = model(batch_poses)
                loss = criterion(pred, batch_joints)
                val_loss += loss.item() * len(batch_poses)

        val_loss /= (n - n_train)
        scheduler.step(val_loss)

        if (epoch + 1) % 10 == 0:
            print(f"  Epoch {epoch+1}/{epochs}: "
                  f"train_loss={train_loss:.6f}, val_loss={val_loss:.6f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            if model_path:
                torch.save({
                    'model_state': model.state_dict(),
                    'pos_mean': pos_mean,
                    'pos_std': pos_std,
                }, model_path)

    print(f"Training complete. Best val loss: {best_val_loss:.6f}")
    if model_path:
        print(f"Model saved to {model_path}")

    return model


# ═══════════════════════════════════════════════════════════════
# Inference
# ═══════════════════════════════════════════════════════════════
_model: Optional['IKNet'] = None
_pos_mean: Optional[np.ndarray] = None
_pos_std: Optional[np.ndarray] = None


def _load_model(model_path: str) -> bool:
    """Load trained model from disk."""
    global _model, _pos_mean, _pos_std

    if not TORCH_AVAILABLE:
        return False

    if not os.path.exists(model_path):
        return False

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    checkpoint = torch.load(model_path, map_location=device)

    _model = IKNet().to(device)
    _model.load_state_dict(checkpoint['model_state'])
    _model.eval()

    _pos_mean = checkpoint['pos_mean']
    _pos_std = checkpoint['pos_std']

    return True


def solve_neural(target_position: np.ndarray,
                 target_orientation: Optional[np.ndarray] = None,
                 model_path: str = None,
                 config: Optional[SolverConfig] = None) -> IKResult:
    """
    Solve IK using the trained neural network.

    Fast inference (<1ms on GPU), but approximate.
    Solution is verified via FK; marked as failed if error > tolerance.

    Args:
        target_position: [x, y, z]
        target_orientation: quaternion [qx, qy, qz, qw] (optional)
        model_path: path to trained model .pt
        config: solver configuration

    Returns:
        IKResult
    """
    if not TORCH_AVAILABLE:
        return IKResult(
            success=False,
            solver_name="neural",
            message="PyTorch not installed"
        )

    if config is None:
        config = SolverConfig()

    t_start = time.perf_counter()

    # Load model if needed
    global _model, _pos_mean, _pos_std
    if _model is None:
        if model_path is None:
            # Default path
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(pkg_dir, 'models', 'neural_ik.pt')

        if not _load_model(model_path):
            return IKResult(
                success=False,
                solver_name="neural",
                solve_time_ms=(time.perf_counter() - t_start) * 1000,
                message=f"Model not found at {model_path}. Run training first.",
            )

    device = next(_model.parameters()).device

    # Build input tensor
    if target_orientation is None:
        target_orientation = np.array([0.0, 0.0, 0.0, 1.0])

    # Normalize position
    pos_norm = (target_position - _pos_mean) / _pos_std
    input_vec = np.concatenate([pos_norm, target_orientation])

    input_tensor = torch.tensor(
        input_vec, dtype=torch.float32
    ).unsqueeze(0).to(device)

    # Inference
    with torch.no_grad():
        normalized_output = _model(input_tensor)
        joint_angles_tensor = _model.denormalize_joints(normalized_output)
        joint_angles = joint_angles_tensor.cpu().numpy().flatten()

    # Clamp to limits
    for j in range(5):
        joint_angles[j] = np.clip(
            joint_angles[j], JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1]
        )

    # Verify via FK
    T = forward_kinematics(joint_angles)
    fk_pos = T[:3, 3]
    pos_error = np.linalg.norm(fk_pos - target_position)

    solve_time = (time.perf_counter() - t_start) * 1000

    success = pos_error < config.tolerance_m * 10  # 10mm default

    return IKResult(
        success=success,
        joint_angles=joint_angles,
        position_error_m=pos_error,
        solve_time_ms=solve_time,
        solver_name="neural",
        message=f"{'Solved' if success else 'High error'}: {pos_error*1000:.1f}mm",
    )


if __name__ == "__main__":
    print("═══ Neural IK Self-Test ═══")

    if not TORCH_AVAILABLE:
        print("❌ PyTorch not installed. Run: pip install torch")
    else:
        print(f"CUDA available: {torch.cuda.is_available()}")
        print("To train: call train_neural_ik()")
        print("To test: call solve_neural() after training")
