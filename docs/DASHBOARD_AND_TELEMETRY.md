# Project ARIA: Dashboard, Telemetry & Human-in-the-Loop Interface

## 1. Overview & Architecture

Project ARIA includes an integrated, real-time web telemetry dashboard located in [`arm_dashboard/`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/). The dashboard provides operators with live situational awareness, multi-camera feeds, joint diagnostics, reasoning chain inspection, and interactive human-in-the-loop approvals.

```
┌─────────────────────────────────────────────────────────────┐
│               ARIA TELEMETRY DASHBOARD (Vite + React)       │
├──────────────┬───────────────────────────────┬──────────────┤
│ Camera Feeds │ 3D Arm Pose & Joint Sliders   │ World Map    │
│ (Overhead &  │ Waist, Shoulder, Elbow,       │ 2D/3D Object │
│  Wrist Live) │ Wrist Pitch, Gripper Effort   │ Scene Graph  │
├──────────────┴───────────────────────────────┼──────────────┤
│ Natural Language Chain-of-Thought Log        │ Health Panel │
│ "1. Locate bottle -> 2. Plan grasp..."       │ Temps & FPS  │
└──────────────────────────────────────────────┴──────────────┘
```

- **Backend:** Python FastAPI running with Uvicorn (`port 8000`), bridging ROS 2 Humble topics directly to HTTP and WebSockets.
- **Frontend:** React 18, Vite, Lucide Icons, and Vanilla CSS glassmorphic components (`port 5173`).
- **Telemetry Frequency:** $20\,\text{Hz}$ via bidirectional WebSockets (`ws://localhost:8000/ws`).

---

## 2. Dashboard Components Breakdown

Located in [`arm_dashboard/frontend/src/components/`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/):

1. **`CameraPanel.jsx` ([`CameraPanel.jsx`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/CameraPanel.jsx)):**  
   Displays live visual feeds from both the overhead workspace camera and the wrist-mounted eye-in-hand camera, overlaid with YOLO bounding boxes, tracking IDs, and AprilTag fiducial markers.

2. **`ArmVisualization.jsx` ([`ArmVisualization.jsx`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/ArmVisualization.jsx)):**  
   Provides interactive 3D rendering of the robot's physical configuration, synchronized with `/joint_states`. Allows operators to inspect forward kinematics and manually manipulate joint angles via sliders in manual mode.

3. **`ChainOfThought.jsx` ([`ChainOfThought.jsx`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/ChainOfThought.jsx)):**  
   Renders the step-by-step cognitive reasoning of the agent stack for each incoming task. Every action is explicitly displayed with its confidence score, planning justification, and execution status:
   ```
   Task: "Put the red bottle in the box"
   [1/8] 🔍 LOCATE: Target 'red bottle' detected at [0.182, 0.045, 0.608]m (Conf: 94%)
   [2/8] 📦 LOCATE: Destination receptacle 'box' confirmed at [0.080, -0.200, 0.620]m
   [3/8] 📐 PLAN: Evaluating analytical IK on M_task (pitch = -1.45 rad) -> Solved (0.08ms)
   [4/8] 🦾 EXECUTE: Quintic trajectory dispatched (duration: 1.82s, peak_j2: 1.18 rad/s)
   [5/8] 🤏 GRIP: Gripper clamped; contact force confirmed (3.8N) -> Workpiece secured
   [6/8] 🚀 TRANSFER: Intermediate clearance waypoint reached -> Approaching box
   [7/8] 🔓 RELEASE: Gripper opened -> Workpiece dropped in receptacle
   [8/8] ✅ VERIFY: Optical inspection confirms box occupancy -> Task COMPLETE
   ```

4. **`HealthPanel.jsx` ([`HealthPanel.jsx`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/HealthPanel.jsx)):**  
   Live telemetry monitoring servo thermal estimates ($T_i^\circ\text{C}$), joint backlash drift ($|\Delta \theta|$), camera frame rates, GPU inference latencies, and active E-STOP safety state.

5. **`WorldMap.jsx` ([`WorldMap.jsx`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/WorldMap.jsx)):**  
   Top-down 2D spatial radar of the workcell, displaying the robot base, conveyor belt bounds, sorting bins, and all tracked entities with their lifecycle states (`Detected`, `Tracked`, `Lost`, `Moved`).

6. **`TaskControl.jsx` ([`TaskControl.jsx`](file:///home/gaminizer/Projects/ARIA/arm_dashboard/frontend/src/components/TaskControl.jsx)):**  
   Input prompt interface for typing natural language commands, triggering named poses (`ready`, `park`, `inspect`), toggling teleoperation teach mode, and emergency stop halt.

---

## 3. Human-in-the-Loop (HITL) Low-Confidence Approvals

When the agent's estimated task success confidence drops below the threshold ($\le 75\%$), or when multiple ambiguous objects exist in the scene, the system pauses execution and prompts the user through an interactive modal in `TaskControl`:

```
┌─────────────────────────────────────────────────────────────┐
│ ⏸ PAUSED — Operator Approval Required                       │
├─────────────────────────────────────────────────────────────┤
│ Instruction: "Grasp the screwdriver and rotate it"          │
│ Reason: In-hand reorientation confidence is 62%.            │
│ Grip slip risk is elevated on polished metallic surface.    │
│                                                             │
│ [ Approve & Execute ]              [ Abort / Re-plan ]      │
└─────────────────────────────────────────────────────────────┘
```

The operator can approve the action via `/aria/approve` or abort via `/aria/reject`.

---

## 4. Starting the Telemetry Dashboard

The dashboard can be launched as part of the full system or as a standalone service:

```bash
# Option 1: Integrated full-system launcher (Simulation + Agents + Dashboard)
./launch.sh

# Option 2: Standalone Dashboard Server (serves built React frontend + API)
./arm_dashboard/launch_dashboard.sh
# (or manually: python3 arm_dashboard/app.py)

# Option 3: React Frontend Development Mode (hot-reloading)
cd arm_dashboard/frontend
npm run dev -- --host
```

The interface is accessible at:
- **Integrated Web Console (Production):** [http://localhost:8000](http://localhost:8000)
- **Interactive REST API (Swagger):** [http://localhost:8000/docs](http://localhost:8000/docs)
- **Real-Time State WebSocket:** `ws://localhost:8000/ws/state` (10 Hz JSON telemetry)
- **Live Camera Stream WebSocket:** `ws://localhost:8000/ws/cameras` (30 FPS dual stream)
- **Vite Dev Server (Optional):** [http://localhost:5173](http://localhost:5173)
