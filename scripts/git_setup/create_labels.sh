#!/bin/bash
# Creates standardized GitHub labels for ARIA.
# Usage: ./create_labels.sh <github_user> <repo_name>

set -euo pipefail
REPO="$1/$2"

create_label() {
  local name=$1
  local color=$2
  local description=$3
  gh label create "$name" \
    --color "$color" \
    --description "$description" \
    --repo "$REPO" \
    --force 2>/dev/null
  echo "  ✅ $name"
}

echo "Creating labels..."

# ── Stage labels ──────────────────────────────────────────────
create_label "stage:1-foundation"          "0075ca" "Stage 1: Gazebo + manual control"
create_label "stage:2-perception"          "0075ca" "Stage 2: IK + vision"
create_label "stage:3a-agents"             "0075ca" "Stage 3a: agents + planning"
create_label "stage:3b-world"              "0075ca" "Stage 3b: world model + skills"
create_label "stage:4-hardware"            "0075ca" "Stage 4: real hardware"
create_label "upgrade:u1-llm"              "7057ff" "Upgrade U1: LLM planning"
create_label "upgrade:u2-perception"       "7057ff" "Upgrade U2: advanced perception"
create_label "upgrade:u3-infra"            "7057ff" "Upgrade U3: infrastructure"

# ── Package labels ────────────────────────────────────────────
create_label "pkg:arm_description"         "e4e669" "URDF, xacro, meshes"
create_label "pkg:arm_control"             "e4e669" "HAL, trajectory, servo"
create_label "pkg:arm_vision"              "e4e669" "cameras, YOLO, depth"
create_label "pkg:arm_ik"                  "e4e669" "IK solvers, benchmark"
create_label "pkg:arm_planner"             "e4e669" "planning, agents, state"
create_label "pkg:arm_agents"              "e4e669" "all 15 agents"
create_label "pkg:arm_vla"                 "e4e669" "VLA interface"
create_label "pkg:arm_learning"            "e4e669" "learning, datasets"
create_label "pkg:arm_dashboard"           "e4e669" "FastAPI + React UI"
create_label "pkg:esp32"                   "e4e669" "firmware, servo sync"

# ── Status labels ─────────────────────────────────────────────
create_label "stage-complete"              "0e8a16" "Stage validated and ready to merge"
create_label "ready-for-merge"             "0e8a16" "PR reviewed and approved"
create_label "in-progress"                 "fbca04" "Actively being worked on"
create_label "blocked"                     "d93f0b" "Cannot proceed, needs resolution"
create_label "needs-hardware"              "d93f0b" "Cannot test without real arm"
create_label "hardware-only"              "d93f0b" "Applies to hardware mode only"

# ── Issue type labels ─────────────────────────────────────────
create_label "bug"                         "d73a4a" "Something is broken"
create_label "enhancement"                 "a2eeef" "New feature or improvement"
create_label "hardware"                    "f9d0c4" "Physical arm or sensor issue"
create_label "urgent"                      "d93f0b" "Needs immediate attention"
create_label "safety"                      "d93f0b" "⚠️ Safety-related"
create_label "calibration"                 "bfd4f2" "Calibration drift or error"
create_label "performance"                 "bfd4f2" "Latency, accuracy, or throughput"
create_label "documentation"              "0075ca" "Docs, README, comments"
create_label "good-first-issue"            "7057ff" "Good for learning the codebase"
create_label "duplicate"                   "cfd3d7" "Already reported"
create_label "wontfix"                     "ffffff" "Will not be addressed"

echo "✅ All labels created"
