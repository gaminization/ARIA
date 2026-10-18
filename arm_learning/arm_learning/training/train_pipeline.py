#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Training Pipeline — behavioural-cloning trainer
═══════════════════════════════════════════════════════════════

Trains a small policy that maps robot observations to actions on
ARIA-collected LeRobot HDF5 datasets (see arm_learning.bag_to_dataset).

This is a REAL training loop: it reads the dataset, runs optimisation,
reports the loss it actually measured, and writes a checkpoint. It never
fabricates metrics.

Dependencies
------------
* ``h5py``  — required to read datasets.
* ``torch`` — required to train. Without it the pipeline reports failure
  honestly instead of pretending to train.

Two training modes are selected automatically from the dataset:
  * state-only  → an MLP over ``observation.state``.
  * image + state → a small CNN over ``observation.images.top`` (+ wrist)
    concatenated with the state vector. Only used when image datasets are
    actually present (ARIA's default ``bag_to_dataset`` output is state-only
    unless FULL-mode bags with camera topics were recorded).

Notes
-----
* The policy this trainer produces is a behaviour-cloning baseline, NOT a
  fine-tuned VLA (OpenVLA/ACT/Pi0). Producing a VLA checkpoint requires the
  backend's own pinned training stack and is out of scope here.
* Success is only claimed when the optimisation ran and a checkpoint was
  written; all metrics come from the real training loop.
═══════════════════════════════════════════════════════════════
"""

import os
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class DatasetError(RuntimeError):
    """Raised when the dataset cannot be used for training."""


class TrainingPipeline:
    """Real (state and/or image) behavioural-cloning trainer."""

    def __init__(self, dataset_path: str, model_name: str = "aria-bc",
                 output_dir: str = ""):
        self.dataset_path = dataset_path
        self.model_name = model_name
        self.output_dir = output_dir or os.path.join(
            os.path.dirname(os.path.abspath(dataset_path)) or '.',
            'checkpoints')
        self.h5py = None
        self.torch = None
        self.tracker = None
        self._check_deps()

    # ── Experiment tracking ───────────────────────────────────
    def _get_tracker(self):
        """
        Lazily build an experiment tracker.

        Backend is selected with ``ARIA_TRACKING_BACKEND``
        (``mlflow`` | ``wandb`` | ``none``). The default is ``none``,
        which writes a local JSON run log and never requires a server.
        Tracking is best-effort: any failure is swallowed so it can
        never break training.
        """
        if self.tracker is None:
            try:
                from arm_learning.experiment_tracker import ExperimentTracker
                backend = getattr(self, '_tracking_backend', '') or os.environ.get(
                    'ARIA_TRACKING_BACKEND', 'none')
                self.tracker = ExperimentTracker(backend=backend)
            except Exception:
                self.tracker = None
        return self.tracker

    # ── Dependency handling ────────────────────────────────────
    def _check_deps(self):
        try:
            import h5py
            self.h5py = h5py
        except ImportError:
            self.h5py = None
        try:
            import torch
            self.torch = torch
        except ImportError:
            self.torch = None

    @property
    def can_train(self) -> bool:
        return self.h5py is not None and self.torch is not None

    # ── Dataset validation ─────────────────────────────────────
    def validate_dataset(self) -> Dict[str, Any]:
        """Inspect the HDF5 dataset structure without loading tensors."""
        results = {
            'valid': False,
            'n_episodes': 0,
            'n_frames_total': 0,
            'has_images': False,
            'has_joints': False,
            'has_actions': False,
            'state_dim': 0,
            'action_dim': 0,
            'issues': [],
        }

        if not os.path.exists(self.dataset_path):
            results['issues'].append(f"Dataset not found: {self.dataset_path}")
            return results

        if self.h5py is None:
            results['issues'].append("h5py not installed (pip install h5py)")
            return results

        try:
            with self.h5py.File(self.dataset_path, 'r') as f:
                state_dim = 0
                action_dim = 0
                if 'episodes' in f:
                    ep_group = f['episodes']
                    results['n_episodes'] = len(ep_group)
                    for ep in ep_group.values():
                        results['n_frames_total'] += int(
                            ep.attrs.get('n_frames', 0))
                        if 'observation.images.top' in ep:
                            results['has_images'] = True
                        if 'observation.state' in ep:
                            results['has_joints'] = True
                            state_dim = max(
                                state_dim,
                                int(ep['observation.state'].shape[-1]))
                        if 'action' in ep:
                            results['has_actions'] = True
                            action_dim = max(
                                action_dim,
                                int(ep['action'].shape[-1]))
                    results['state_dim'] = state_dim
                    results['action_dim'] = action_dim
                else:
                    results['issues'].append("No 'episodes' group in HDF5 file")

                results['valid'] = bool(
                    results['has_joints'] and results['has_actions']
                    and results['n_frames_total'] > 0)

            if not results['valid'] and not results['has_actions']:
                results['issues'].append(
                    "Dataset has no 'action' group — it cannot be used for "
                    "behaviour cloning (needs recorded commands, not just "
                    "observed states)")
        except Exception as e:  # noqa: BLE001 — surface real read errors
            results['issues'].append(f"Error reading HDF5: {e}")

        return results

    # ── Dataset loading ────────────────────────────────────────
    def load_dataset(self) -> Dict[str, np.ndarray]:
        """Load states/actions (and images if present) from all episodes.

        Raises DatasetError when the file is missing, unreadable, has no
        usable action data, or the modal dimensions disagree.
        """
        if self.h5py is None:
            raise DatasetError("h5py not installed (pip install h5py)")
        if not os.path.exists(self.dataset_path):
            raise DatasetError(f"Dataset not found: {self.dataset_path}")

        states: List[np.ndarray] = []
        actions: List[np.ndarray] = []
        images: List[np.ndarray] = []

        with self.h5py.File(self.dataset_path, 'r') as f:
            if 'episodes' not in f:
                raise DatasetError("No 'episodes' group in HDF5 file")
            for ep in f['episodes'].values():
                if 'observation.state' not in ep or 'action' not in ep:
                    continue
                s = np.asarray(ep['observation.state'], dtype=np.float32)
                a = np.asarray(ep['action'], dtype=np.float32)
                n = min(len(s), len(a))
                if n == 0:
                    continue
                s, a = s[:n], a[:n]
                if s.ndim == 1:
                    s = s[None, :]
                if a.ndim == 1:
                    a = a[None, :]
                # Guard against state==action circular datasets (no learning
                # signal): the loader accepts them but the caller can warn.
                states.append(s)
                actions.append(a)
                if 'observation.images.top' in ep:
                    imgs = np.asarray(ep['observation.images.top'])
                    if imgs.shape[0] >= n:
                        images.append(imgs[:n])

        if not states:
            raise DatasetError(
                "No usable (observation.state, action) pairs found")

        state_dim = states[0].shape[-1]
        action_dim = actions[0].shape[-1]
        for s in states:
            if s.shape[-1] != state_dim:
                raise DatasetError(
                    "Inconsistent state dimension across episodes: "
                    f"{s.shape[-1]} != {state_dim}")
        for a in actions:
            if a.shape[-1] != action_dim:
                raise DatasetError(
                    "Inconsistent action dimension across episodes: "
                    f"{a.shape[-1]} != {action_dim}")

        data = {
            'state': np.concatenate(states, axis=0),
            'action': np.concatenate(actions, axis=0),
        }
        if images and len(images) == len(states):
            data['image'] = np.concatenate(images, axis=0)
        return data

    # ── Model ──────────────────────────────────────────────────
    def _build_model(self, state_dim: int, action_dim: int,
                     image_shape: Optional[Tuple[int, int, int]] = None):
        torch = self.torch
        nn = torch.nn

        class PolicyNet(nn.Module):
            def __init__(self, s_dim, a_dim, img_shape):
                super().__init__()
                self.img_shape = img_shape
                feat_dim = s_dim
                if img_shape is not None:
                    c, h, w = img_shape
                    self.encoder = nn.Sequential(
                        nn.Conv2d(c, 16, 5, stride=2, padding=2),
                        nn.ReLU(),
                        nn.Conv2d(16, 32, 3, stride=2, padding=1),
                        nn.ReLU(),
                        nn.AdaptiveAvgPool2d((4, 4)),
                        nn.Flatten(),
                    )
                    feat_dim += 32 * 4 * 4
                self.mlp = nn.Sequential(
                    nn.Linear(feat_dim, 256), nn.ReLU(),
                    nn.Linear(256, 256), nn.ReLU(),
                    nn.Linear(256, a_dim),
                )

            def forward(self, state, image=None):
                if self.img_shape is not None and image is not None:
                    img = image.permute(0, 3, 1, 2).float() / 255.0
                    x = torch.cat([self.encoder(img), state], dim=-1)
                else:
                    x = state
                return self.mlp(x)

        return PolicyNet(state_dim, action_dim, image_shape)

    # ── Training ───────────────────────────────────────────────
    def train(self, epochs: int = 20, batch_size: int = 64,
              lr: float = 1e-3, val_split: float = 0.2,
              min_frames: int = 50, seed: int = 0) -> Dict[str, Any]:
        """Run a real training loop and write a checkpoint.

        Returns a result dict. ``result['success']`` is True only when the
        optimisation actually ran and a checkpoint was saved.
        """
        if not self.can_train:
            missing = []
            if self.h5py is None:
                missing.append('h5py')
            if self.torch is None:
                missing.append('torch')
            return {
                'success': False,
                'trained': False,
                'error': f"Missing training dependencies: {', '.join(missing)}",
            }

        try:
            data = self.load_dataset()
        except DatasetError as e:
            return {'success': False, 'trained': False, 'error': str(e)}

        state = data['state']
        action = data['action']
        n_frames = len(state)
        if n_frames < min_frames:
            return {
                'success': False,
                'trained': False,
                'error': (
                    f"Dataset has only {n_frames} usable frames "
                    f"(need >= {min_frames}); refusing to train on "
                    "insufficient data"),
                'n_frames': n_frames,
            }

        torch = self.torch
        torch.manual_seed(seed)
        rng = np.random.default_rng(seed)

        # Train/val split.
        idx = rng.permutation(n_frames)
        n_val = max(1, int(n_frames * val_split))
        val_idx, train_idx = idx[:n_val], idx[n_val:]

        image = data.get('image')
        image_shape = None
        if image is not None and image.ndim == 4:
            image_shape = (
                image.shape[3], image.shape[1], image.shape[2])  # (C, H, W)

        state_mean = state[train_idx].mean(axis=0, keepdims=True)
        state_std = state[train_idx].std(axis=0, keepdims=True) + 1e-6
        action_mean = action[train_idx].mean(axis=0, keepdims=True)
        action_std = action[train_idx].std(axis=0, keepdims=True) + 1e-6

        def to_tensor(s_idx):
            s = (state[s_idx] - state_mean) / state_std
            a = (action[s_idx] - action_mean) / action_std
            im = image[s_idx] if image is not None else None
            return torch.as_tensor(s, dtype=torch.float32), \
                torch.as_tensor(a, dtype=torch.float32), \
                (torch.as_tensor(im) if im is not None else None)

        s_tr, a_tr, im_tr = to_tensor(train_idx)
        s_va, a_va, im_va = to_tensor(val_idx)

        model = self._build_model(
            state.shape[-1], action.shape[-1], image_shape)
        optimiser = torch.optim.Adam(model.parameters(), lr=lr)
        loss_fn = torch.nn.MSELoss()

        os.makedirs(self.output_dir, exist_ok=True)
        ckpt_path = os.path.join(self.output_dir, 'policy_best.pt')

        history: List[Dict[str, float]] = []
        best_val = float('inf')
        t_start = time.time()

        # ── Experiment tracking (best-effort; never breaks training) ──
        tracker = self._get_tracker()
        if tracker is not None:
            try:
                tracker.start_training_run(
                    f"train_{self.model_name}_{int(t_start)}",
                    {
                        'dataset': self.dataset_path,
                        'model_name': self.model_name,
                        'epochs': epochs,
                        'batch_size': batch_size,
                        'lr': lr,
                        'val_split': val_split,
                        'min_frames': min_frames,
                        'seed': seed,
                        'state_dim': int(state.shape[-1]),
                        'action_dim': int(action.shape[-1]),
                        'n_frames': int(n_frames),
                        'uses_images': image_shape is not None,
                    })
            except Exception:
                tracker = None

        for epoch in range(1, epochs + 1):
            model.train()
            perm = torch.randperm(len(s_tr))
            epoch_loss = 0.0
            n_batches = 0
            for start in range(0, len(s_tr), batch_size):
                bidx = perm[start:start + batch_size]
                optimiser.zero_grad()
                pred = model(
                    s_tr[bidx], im_tr[bidx] if im_tr is not None else None)
                loss = loss_fn(pred, a_tr[bidx])
                loss.backward()
                optimiser.step()
                epoch_loss += float(loss.item())
                n_batches += 1
            train_loss = epoch_loss / max(n_batches, 1)

            model.eval()
            with torch.no_grad():
                val_pred = model(
                    s_va, im_va if im_va is not None else None)
                val_loss = float(loss_fn(val_pred, a_va).item())

            history.append({'epoch': epoch,
                            'train_loss': train_loss,
                            'val_loss': val_loss})

            if tracker is not None:
                try:
                    tracker.log_training_epoch(
                        epoch, {'train_loss': train_loss, 'val_loss': val_loss})
                except Exception:
                    pass

            if val_loss < best_val:
                best_val = val_loss
                torch.save({
                    'model_state': model.state_dict(),
                    'state_mean': state_mean,
                    'state_std': state_std,
                    'action_mean': action_mean,
                    'action_std': action_std,
                    'state_dim': int(state.shape[-1]),
                    'action_dim': int(action.shape[-1]),
                    'image_shape': image_shape,
                    'val_loss': val_loss,
                    'epoch': epoch,
                }, ckpt_path)

        duration_s = time.time() - t_start

        if tracker is not None:
            try:
                tracker.finish_training_run(
                    {
                        'final_train_loss': history[-1]['train_loss'],
                        'final_val_loss': history[-1]['val_loss'],
                        'best_val_loss': best_val,
                        'n_train': int(len(train_idx)),
                        'n_val': int(len(val_idx)),
                        'duration_s': duration_s,
                    },
                    artifacts=[ckpt_path])
            except Exception:
                pass

        return {
            'success': True,
            'trained': True,
            'checkpoint': ckpt_path,
            'epochs': epochs,
            'batch_size': batch_size,
            'n_frames': n_frames,
            'n_train': int(len(train_idx)),
            'n_val': int(len(val_idx)),
            'uses_images': image_shape is not None,
            'final_train_loss': history[-1]['train_loss'],
            'final_val_loss': history[-1]['val_loss'],
            'best_val_loss': best_val,
            'history': history,
            'duration_s': time.time() - t_start,
        }


def main():
    """CLI entry point — validate and (really) train a policy."""
    import argparse
    import json as _json

    parser = argparse.ArgumentParser(description='ARIA training pipeline')
    parser.add_argument('--dataset', required=True, help='Path to HDF5 dataset')
    parser.add_argument('--model', default='aria-bc',
                        help='Model/run name (for bookkeeping)')
    parser.add_argument('--output-dir', default='',
                        help='Where to write checkpoints')
    parser.add_argument('--epochs', type=int, default=20)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--min-frames', type=int, default=50)
    parser.add_argument('--check-only', action='store_true',
                        help='Only validate the dataset')
    parser.add_argument('--tracking-backend',
                        default=os.environ.get('ARIA_TRACKING_BACKEND', 'none'),
                        choices=['mlflow', 'wandb', 'none'],
                        help='Experiment tracker backend (default: none = local JSON)')
    args = parser.parse_args()

    pipeline = TrainingPipeline(
        args.dataset, args.model, output_dir=args.output_dir)
    pipeline._tracking_backend = args.tracking_backend

    print("ARIA Training Pipeline")
    print(f"  Dataset: {args.dataset}")
    print(f"  Model:   {args.model}")
    print(f"  Tracking: {args.tracking_backend}")
    print(f"  Can train (torch+h5py): {pipeline.can_train}")
    print()

    val = pipeline.validate_dataset()
    print("Dataset validation:")
    print(f"  Valid: {val['valid']}")
    print(f"  Episodes: {val['n_episodes']}")
    print(f"  Total frames: {val['n_frames_total']}")
    print(f"  Has images: {val['has_images']}")
    print(f"  state_dim={val['state_dim']} action_dim={val['action_dim']}")
    if val['issues']:
        print(f"  Issues: {val['issues']}")
    print()

    if args.check_only:
        return

    result = pipeline.train(
        epochs=args.epochs, batch_size=args.batch_size,
        lr=args.lr, min_frames=args.min_frames)
    print("Training result:")
    print(_json.dumps(
        {k: v for k, v in result.items() if k != 'history'}, indent=2))
    if result.get('history'):
        print(f"  last epoch: {result['history'][-1]}")


if __name__ == '__main__':
    main()
