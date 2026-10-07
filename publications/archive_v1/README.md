# ARIA — IEEE Transactions Journal Paper

This directory contains the complete, publication-grade IEEE journal paper manuscript for **Project ARIA** (*Adaptive Robotic Intelligence Architecture / Autonomous Reasoning & Interaction Agent*), formatted using the official `IEEEtran.cls` (v1.8b) journal template.

---

## Deliverables Summary

| File | Description |
|------|-------------|
| [`aria_journal_paper.tex`](file:///home/gaminizer/Projects/ARIA/paper/aria_journal_paper.tex) | Main LaTeX manuscript (11 sections, mathematical derivations, tables, algorithms, CoT stream, appendices, and author biography). |
| [`references.bib`](file:///home/gaminizer/Projects/ARIA/paper/references.bib) | Complete BibTeX bibliography containing all 136 papers from the ARIA Literature Compendium across 9 research clusters. |
| [`IEEEtran.cls`](file:///home/gaminizer/Projects/ARIA/paper/IEEEtran.cls) | Official IEEE Transactions LaTeX document class file (v1.8b). |
| [`IEEEtran.bst`](file:///home/gaminizer/Projects/ARIA/paper/IEEEtran.bst) | Official IEEE bibliography style file for BibTeX. |
| [`generate_figures.py`](file:///home/gaminizer/Projects/ARIA/paper/generate_figures.py) | Python script using `matplotlib` and `PIL` to regenerate all 9 publication-grade figures. |
| [`compile.sh`](file:///home/gaminizer/Projects/ARIA/paper/compile.sh) | Automated compilation script (`pdflatex` + `bibtex`). |
| [`figures/`](file:///home/gaminizer/Projects/ARIA/paper/figures/) | Directory containing all 9 high-resolution figures (300 DPI). |

---

## Figure Catalog

1. **`fig1_system_architecture.png`**: Multi-tiered decentralized cognitive architecture across 6 tiers (User Interface, LLM Planning, Unified State Bus, Perception, World Model/Skills, Hardware/Digital Twin, Metamonitoring).
2. **`fig2_kinematics_dh.png`**: Manipulator kinematic structure, link coordinate frames, and Modified Denavit-Hartenberg (DH) parameter table.
3. **`fig3_perception_pipeline.png`**: Overhead Logitech C270 camera view, eye-in-hand ESP32-CAM view, and Depth-Anything v2 monocular metric 3D depth reconstruction with table-plane homography ($RMSE = 8.2\text{ mm}$).
4. **`fig4_planning_hitl.png`**: Hierarchical task decomposition flowchart, Tree-of-Thoughts (ToT) search, confidence-gated Human-in-the-Loop (HITL) approval thresholding ($\tau = 0.70$), and live Chain-of-Thought (CoT) terminal stream.
5. **`fig5_inhand_manipulation.png`**: Four IMU-guided in-hand manipulation primitives (`rotate`, `reposition_grip`, `flip`, `slide_to_tip`) executed on a 1-DoF parallel gripper.
6. **`fig6_ik_benchmark.png`**: Inverse Kinematics solver benchmark across 200 reachable poses comparing solve latency ($0.08\text{ ms}$ for ARIA analytical vs $18.5\text{ ms}$ for IKPy), position error ($0.02\text{ mm}$), and success rate ($100\%$).
7. **`fig7_vla_benchmark.png`**: Standardized 10-task VLA benchmark comparing Rule-Based Baseline ($89\%$), LeRobot ACT ($72\%$), OpenVLA ($68\%$), and $\pi_0$ ($61\%$) across Easy, Medium, and Hard manipulation tasks.
8. **`fig8_conveyor_sorting.png`**: Multi-panel chronological montage demonstrating end-to-end industrial workcell conveyor belt sorting strictly using the eye-in-hand gripper camera.
9. **`fig9_dashboard_ui.png`**: ARIA Control Center web dashboard layout (3D arm visualizer, camera streams, task control, world model semantic map, servo health monitors).

---

## How to Compile

```bash
cd /home/gaminizer/Projects/ARIA/paper
./compile.sh
```

Or manually:
```bash
pdflatex aria_journal_paper.tex
bibtex aria_journal_paper
pdflatex aria_journal_paper.tex
pdflatex aria_journal_paper.tex
```

If TeX Live is not installed on your system:
```bash
sudo apt update
sudo apt install texlive-latex-base texlive-latex-extra texlive-fonts-recommended texlive-bibtex-extra
```
