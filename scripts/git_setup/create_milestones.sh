#!/bin/bash
set -euo pipefail
REPO="$1/$2"

create_milestone() {
  local title=$1
  local description=$2
  local due=$3
  gh api \
    --method POST \
    "/repos/$REPO/milestones" \
    -f title="$title" \
    -f description="$description" \
    -f due_on="${due}T00:00:00Z" \
    --silent 2>/dev/null && echo "  ✅ $title" || echo "  ⚠️  $title (may already exist)"
}

echo "Creating milestones..."

# Calculate relative dates (adjust to your actual schedule)
YEAR=$(date +%Y)

create_milestone \
  "v0.1.0 — Stage 1: Foundation" \
  "Gazebo simulation with full manual joint control" \
  "${YEAR}-$(date -d '+2 weeks' +%m-%d)"

create_milestone \
  "v0.2.0 — Stage 2: IK + Perception" \
  "IK benchmark, depth estimation, first autonomous grasp" \
  "${YEAR}-$(date -d '+6 weeks' +%m-%d)"

create_milestone \
  "v0.3.0 — Stage 3a: Agents + Planning" \
  "All 15 agents, NLP planning, chain of thought" \
  "${YEAR}-$(date -d '+9 weeks' +%m-%d)"

create_milestone \
  "v0.4.0 — Stage 3b: World Model + Skills" \
  "World model, skills, dashboard, VLA" \
  "${YEAR}-$(date -d '+12 weeks' +%m-%d)"

create_milestone \
  "v1.0.0 — Stage 4: Hardware" \
  "First real hardware deployment" \
  "${YEAR}-$(date -d '+16 weeks' +%m-%d)"

create_milestone \
  "v1.1.0 — Upgrade U1: LLM" \
  "Ollama, ReAct, Tree-of-Thought, episodic memory" \
  "${YEAR}-$(date -d '+20 weeks' +%m-%d)"

create_milestone \
  "v1.2.0 — Upgrade U2: Advanced Perception" \
  "6D pose, SAM2, transparent objects, materials" \
  "${YEAR}-$(date -d '+24 weeks' +%m-%d)"

create_milestone \
  "v1.3.0 — Upgrade U3: Infrastructure" \
  "Docker, CI/CD, MLflow, bag pipeline, object DB" \
  "${YEAR}-$(date -d '+26 weeks' +%m-%d)"

echo "✅ All milestones created"
