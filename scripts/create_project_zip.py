#!/usr/bin/env python3
"""
scripts/create_project_zip.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Packages Project ARIA into a clean, comprehensive distribution archive.
Includes all source code, ROS 2 packages, firmware, hardware descriptions,
simulation worlds, empirical Phase 0-2 benchmark data and scripts,
IEEE T-RO publication materials, and system configurations.
Excludes build caches, virtual environments, raw git metadata, external clones,
and oversized pretrained foundation model checkpoints (OpenVLA-7B).
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
import zipfile
import time
from datetime import datetime, timezone

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DIST_DIR = os.path.join(WORKSPACE_ROOT, "dist")
os.makedirs(DIST_DIR, exist_ok=True)

OUTPUT_ZIP = os.path.join(DIST_DIR, "aria_project_v2_updated.zip")
ROOT_LINK_ZIP = os.path.join(WORKSPACE_ROOT, "aria_project_v2_updated.zip")

EXCLUDE_DIRS = {
    '.git',
    'build',
    'install',
    'log',
    'dist',
    'aria_venv',
    '.venv',
    'venv',
    '.pytest_cache',
    '__pycache__',
    'node_modules',
}

EXCLUDE_REL_PREFIXES = [
    'models/openvla-7b',
    'third_party/repos',
    'docs/literature/external_papers',
    'arm_dashboard/frontend/node_modules',
    'data/real/rosbags',
]

EXCLUDE_FILES = {
    'ik_benchmark_cartesian_100k.csv',
    'ik_benchmark_fk_100k.csv',
    'ik_benchmark_5d_fk_10k.csv',
    'ik_benchmark_5d_cartesian_10k.csv',
    'sam2.1_hiera_tiny.pt',
    'sam2_hiera_tiny.pt',
}

EXCLUDE_EXTS = {'.pyc', '.so', '.o', '.a', '.d'}

def build_zip():
    print(f"Creating project distribution archive: {OUTPUT_ZIP}")
    start_time = time.time()
    
    # Collect files
    file_list = []
    total_uncompressed = 0
    
    for root, dirs, files in os.walk(WORKSPACE_ROOT):
        # Filter directories in-place
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
        rel_root = os.path.relpath(root, WORKSPACE_ROOT)
        if any(rel_root == p or rel_root.startswith(p + '/') for p in EXCLUDE_REL_PREFIXES):
            dirs[:] = []
            continue
            
        for f in files:
            if f in EXCLUDE_FILES:
                continue
            ext = os.path.splitext(f)[1]
            if ext in EXCLUDE_EXTS:
                continue
            if f.endswith('.zip') or f.startswith('zi'):
                continue
            fp = os.path.join(root, f)
            if os.path.islink(fp):
                continue
            try:
                sz = os.path.getsize(fp)
                rel_path = os.path.relpath(fp, WORKSPACE_ROOT)
                file_list.append((fp, rel_path, sz))
                total_uncompressed += sz
            except OSError:
                pass

    print(f"Collected {len(file_list)} files ({total_uncompressed / (1024*1024):.2f} MB uncompressed)")
    
    # Write zip file
    with zipfile.ZipFile(OUTPUT_ZIP, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for idx, (abs_path, rel_path, sz) in enumerate(file_list, 1):
            if idx % 200 == 0 or idx == len(file_list):
                print(f"  [{idx}/{len(file_list)}] Archiving {rel_path}...")
            zf.write(abs_path, arcname=rel_path)
            
    compressed_size = os.path.getsize(OUTPUT_ZIP)
    elapsed = time.time() - start_time
    print(f"\nArchive successfully generated!")
    print(f"  Output path: {OUTPUT_ZIP}")
    print(f"  Total files: {len(file_list)}")
    print(f"  Uncompressed size: {total_uncompressed / (1024*1024):.2f} MB")
    print(f"  Compressed size:   {compressed_size / (1024*1024):.2f} MB")
    print(f"  Time taken:        {elapsed:.2f} s")
    
    # Also update/create link or copy at workspace root and for Claude export
    for target in [ROOT_LINK_ZIP, os.path.join(DIST_DIR, "aria_project_for_claude.zip")]:
        if os.path.exists(target):
            try:
                os.remove(target)
            except OSError:
                pass
        try:
            os.link(OUTPUT_ZIP, target)
            print(f"  Hardlink created:  {target}")
        except OSError:
            pass

if __name__ == "__main__":
    build_zip()
