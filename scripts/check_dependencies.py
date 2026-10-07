#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
Project ARIA — Comprehensive Dependency & Environment Validator
═══════════════════════════════════════════════════════════════
Validates the entire hardware, system, ROS 2, Python, model,
and web dashboard environment for total reproducibility.

Usage:
  python3 scripts/check_dependencies.py
═══════════════════════════════════════════════════════════════
"""

import sys
import os
import shutil
import subprocess
import importlib

# ANSI Colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"

def print_header(title: str):
    print(f"\n{BOLD}{BLUE}════ {title.upper()} ════{RESET}")

def check_item(name: str, passed: bool, details: str = "", critical: bool = True) -> bool:
    tag = f"{GREEN}[PASS]{RESET}" if passed else (f"{RED}[FAIL]{RESET}" if critical else f"{YELLOW}[WARN]{RESET}")
    print(f"  {tag} {BOLD}{name:<32}{RESET} : {details}")
    return passed or not critical

def main():
    print(f"\n{BOLD}═══════════════════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}  🤖 Project ARIA: Autonomous System Environment Validator      {RESET}")
    print(f"{BOLD}═══════════════════════════════════════════════════════════════{RESET}")

    all_passed = True

    # 1. System & OS
    print_header("1. Operating System & Platform")
    py_ver = sys.version_info
    py_ok = (py_ver.major == 3 and py_ver.minor >= 10)
    all_passed &= check_item("Python 3.10+", py_ok, f"{sys.version.split()[0]} ({sys.executable})")

    platform_str = sys.platform
    all_passed &= check_item("Host Linux OS", platform_str.startswith("linux"), f"Platform: {platform_str}")

    # 2. Compute & GPU Acceleration
    print_header("2. Edge Compute & GPU Acceleration (RTX 5060 Envelope)")
    torch_available = False
    cuda_available = False
    vram_gb = 0.0
    device_name = "N/A"
    try:
        import torch
        torch_available = True
        cuda_available = torch.cuda.is_available()
        if cuda_available:
            device_name = torch.cuda.get_device_name(0)
            vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024**3)
    except Exception as e:
        pass

    all_passed &= check_item("PyTorch Framework", torch_available, f"PyTorch {getattr(torch, '__version__', 'Not Found')}")
    check_item("CUDA GPU Acceleration", cuda_available, f"{device_name} ({vram_gb:.2f} GB VRAM)", critical=False)
    if cuda_available and vram_gb < 7.0:
        print(f"      {YELLOW}Note: Recommended VRAM is >= 8.0 GB for full parallel VLA + Depth-Anything stack.{RESET}")

    # 3. ROS 2 Environment
    print_header("3. ROS 2 Infrastructure")
    ros_distro = os.environ.get("ROS_DISTRO", "none")
    ros_ok = (ros_distro.lower() == "humble")
    check_item("ROS 2 Distribution", ros_ok, f"Detected: '{ros_distro}' (Expected: 'humble')", critical=False)

    rclpy_ok = False
    try:
        import rclpy
        rclpy_ok = True
    except ImportError:
        pass
    check_item("rclpy Python Bindings", rclpy_ok, "Available" if rclpy_ok else "Not sourced (run: source /opt/ros/humble/setup.bash)", critical=False)

    gazebo_path = shutil.which("gazebo") or shutil.which("gzserver")
    check_item("Gazebo Classic 11", gazebo_path is not None, f"Binary: {gazebo_path or 'Not Found'}", critical=False)

    # 4. Core Robotics & Kinematics Packages
    print_header("4. Core Robotics & Kinematics")
    core_pkgs = [
        ("numpy", "NumPy Numerical Computing"),
        ("scipy", "SciPy Optimization & Spatial"),
        ("sympy", "SymPy Symbolic Mathematics"),
        ("spatialmath", "Spatial Math Python (SE3/SO3)"),
        ("roboticstoolbox", "Robotics Toolbox for Python"),
        ("ikpy", "IKPy Inverse Kinematics"),
        ("trimesh", "Trimesh 3D Geometry Processing"),
        ("yaml", "PyYAML Configuration Parser"),
        ("serial", "pySerial Hardware Bus Bridge"),
    ]
    for mod, label in core_pkgs:
        ok = False
        ver = "Not installed"
        try:
            m = importlib.import_module(mod)
            ok = True
            ver = getattr(m, "__version__", "OK")
        except ImportError:
            pass
        all_passed &= check_item(label, ok, f"v{ver}")

    # 5. Vision & Depth Foundation Models
    print_header("5. Vision & Depth Grounding (No-Depth-Cam Pipeline)")
    vision_pkgs = [
        ("cv2", "OpenCV Computer Vision (cv2)"),
        ("PIL", "Pillow Imaging Library"),
        ("ultralytics", "Ultralytics YOLOv8 Instance Detection"),
        ("timm", "PyTorch Image Models (timm)"),
        ("transformers", "HuggingFace Transformers (OpenVLA)"),
    ]
    for mod, label in vision_pkgs:
        ok = False
        ver = "Not installed"
        try:
            m = importlib.import_module(mod)
            ok = True
            ver = getattr(m, "__version__", "OK")
        except ImportError:
            pass
        all_passed &= check_item(label, ok, f"v{ver}")

    # Check SAM2
    sam2_ok = False
    try:
        import sam2
        sam2_ok = True
    except ImportError:
        pass
    check_item("SAM2 Segment Anything", sam2_ok, "Installed" if sam2_ok else "Optional (git+https://github.com/facebookresearch/segment-anything-2.git)", critical=False)

    # 6. Web Dashboard & Telemetry Backend
    print_header("6. Operator Dashboard & WebSockets Telemetry")
    dashboard_pkgs = [
        ("fastapi", "FastAPI Asynchronous Web Framework"),
        ("uvicorn", "Uvicorn ASGI Web Server"),
        ("websockets", "WebSockets Real-Time State Bus"),
        ("pydantic", "Pydantic Schema Validation"),
    ]
    for mod, label in dashboard_pkgs:
        ok = False
        ver = "Not installed"
        try:
            m = importlib.import_module(mod)
            ok = True
            ver = getattr(m, "__version__", "OK")
        except ImportError:
            pass
        all_passed &= check_item(label, ok, f"v{ver}")

    node_path = shutil.which("node")
    npm_path = shutil.which("npm")
    check_item("Node.js Runtime", node_path is not None, f"{node_path or 'Not installed'} (Required for React frontend)", critical=False)
    check_item("NPM Package Manager", npm_path is not None, f"{npm_path or 'Not installed'}", critical=False)

    # 7. Model Weights in models/
    print_header("7. Local Neural Model Weights (models/)")
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    models_dir = os.path.join(root_dir, "models")
    weights = [
        ("yolov8m.pt", "YOLOv8 Medium Object Detector", 45.0),
        ("yolov8n.pt", "YOLOv8 Nano Fast Detector", 6.0),
        ("sam2.1_hiera_tiny.pt", "SAM2.1 Hiera Tiny Mask Segmenter", 140.0),
        ("sam2_hiera_tiny.pt", "SAM2 Hiera Tiny Segmenter", 140.0),
    ]
    for filename, desc, min_mb in weights:
        fpath = os.path.join(models_dir, filename)
        exists = os.path.exists(fpath)
        sz_mb = (os.path.getsize(fpath) / (1024**2)) if exists else 0.0
        check_item(desc, exists and sz_mb >= min_mb, f"{filename} ({sz_mb:.1f} MB)" if exists else "Missing from models/", critical=False)

    # 8. Summary
    print("\n═══════════════════════════════════════════════════════════════")
    if all_passed:
        print(f"{GREEN}{BOLD}  ✅ ARIA ENVIRONMENT VALIDATION: ALL CRITICAL SYSTEMS READY{RESET}")
        print("  The project is fully configured and ready for simulation & execution.")
    else:
        print(f"{RED}{BOLD}  ❌ ARIA ENVIRONMENT VALIDATION: CRITICAL DEPENDENCIES MISSING{RESET}")
        print("  Please run: pip install -r requirements.txt")
    print("═══════════════════════════════════════════════════════════════\n")

if __name__ == "__main__":
    main()
