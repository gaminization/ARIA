# NVIDIA Container Toolkit Setup for ARIA

ARIA requires GPU access inside Docker containers for real-time inference
(YOLO, Depth-Anything, SAM2, FoundationPose).

## Prerequisites

- NVIDIA GPU with driver ≥ 535 (RTX 5060 ships with 565+)
- Docker Engine ≥ 24.0
- Pop!_OS 22.04 / Ubuntu 22.04

## Installation

```bash
# 1. Add NVIDIA Container Toolkit repository
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L "https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list" \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 3. Configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4. Verify
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```

## Expected Output

```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 565.xx.xx    Driver Version: 565.xx.xx    CUDA Version: 12.8                |
|--------------------------------------------+----------------------+---------------------+
| GPU  Name       Persistence-M | Bus-Id     Disp.A | Volatile Uncorr. ECC               |
| Fan  Temp   Perf Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M.           |
|=============================================+======================+====================|
|   0  NVIDIA GeForce RTX 5060     Off | 00000000:01:00.0 On |                  N/A        |
|  0%   35C    P8    10W / 140W |   300MiB /  8192MiB |      0%      Default             |
+--------------------------------------------+----------------------+---------------------+
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `docker: Error response from daemon: could not select device driver` | Install nvidia-container-toolkit (step 2) |
| `nvidia-smi not found` inside container | Restart Docker: `sudo systemctl restart docker` |
| Permission denied on `/dev/nvidia*` | Add user to docker group: `sudo usermod -aG docker $USER` |
| Pop!_OS specific: driver not loading | `sudo apt install system76-driver-nvidia` |
| Compose v1 doesn't support `runtime: nvidia` | Upgrade to Compose v2: `sudo apt install docker-compose-plugin` |

## Docker Compose GPU Config

The `docker-compose.yml` uses `runtime: nvidia`. For older Docker versions,
you may need to set the default runtime in `/etc/docker/daemon.json`:

```json
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
```

Then restart Docker:
```bash
sudo systemctl restart docker
```
