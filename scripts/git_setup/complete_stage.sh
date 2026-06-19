#!/bin/bash
# Run when a stage is fully validated and ready to merge.
# Usage: ./complete_stage.sh <stage_branch> <tag>
# Example: ./complete_stage.sh stage/1-foundation v0.1.0

set -euo pipefail

STAGE_BRANCH=$1
TAG=$2

STAGE_MAP=(
  "stage/1-foundation:Stage 1: Gazebo Foundation + Manual Control"
  "stage/2-ik-perception:Stage 2: IK Solvers + Perception Pipeline"
  "stage/3a-agents-planning:Stage 3a: Core Agents + NLP Planning"
  "stage/3b-world-model-skills:Stage 3b: World Model + Skills + Dashboard"
  "stage/4-hardware:Stage 4: Hardware Bridge + Servo Sync"
  "upgrade/u1-llm-planning:Upgrade U1: LLM Planning + ReAct"
  "upgrade/u2-advanced-perception:Upgrade U2: 6D Pose + SAM2 + Materials"
  "upgrade/u3-infrastructure:Upgrade U3: Docker + CI/CD + Object DB"
)

# Find description for this stage
STAGE_DESC=""
for entry in "${STAGE_MAP[@]}"; do
  if [[ "$entry" == "$STAGE_BRANCH:"* ]]; then
    STAGE_DESC="${entry#*:}"
    break
  fi
done

[[ -z "$STAGE_DESC" ]] && STAGE_DESC="$STAGE_BRANCH"

echo "════════════════════════════════════════"
echo " Completing: $STAGE_DESC"
echo " Tag:        $TAG"
echo "════════════════════════════════════════"

# ── Pre-flight checks ────────────────────────────────────────────
echo ""
echo "Pre-flight checks..."

# Verify on correct branch
CURRENT=$(git branch --show-current)
if [ "$CURRENT" != "$STAGE_BRANCH" ]; then
  echo "❌ Must be on $STAGE_BRANCH (currently on $CURRENT)"
  exit 1
fi

# Verify working tree is clean
if ! git diff-index --quiet HEAD --; then
  echo "❌ Uncommitted changes. Commit or stash first."
  git status --short
  exit 1
fi

# Verify branch is pushed
git fetch origin "$STAGE_BRANCH" --quiet
LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse "origin/$STAGE_BRANCH" 2>/dev/null || echo "none")
if [ "$LOCAL" != "$REMOTE" ]; then
  echo "❌ Branch not fully pushed. Run: git push"
  exit 1
fi

echo "✅ Pre-flight checks passed"

# ── Prompt for validation confirmation ──────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " VALIDATION CHECKLIST — $STAGE_DESC"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

case "$STAGE_BRANCH" in
  "stage/1-foundation")
    CHECKS=(
      "All 12 validate_stage1.py tests pass (✅ green)"
      "Arm spawns in Gazebo without errors"
      "All 6 joints respond to keyboard control"
      "Joint limits enforced (waist cannot exceed ±90°)"
      "Gripper opens and closes"
      "Both cameras publishing at expected fps"
      "E-stop activates and releases cleanly"
      "RViz shows correct TF tree"
    )
    ;;
  "stage/2-ik-perception")
    CHECKS=(
      "All 7 validate_stage2.py tests pass"
      "IK benchmark complete, solver auto-selected and saved"
      "Depth accuracy within tolerance (print MAE)"
      "YOLO detecting all 6 sim objects"
      "Coordinate transform XY error < 10mm"
      "Full grasp pipeline picks red cube"
      "Cable-aware planning avoids cable zones"
    )
    ;;
  "stage/3a-agents-planning")
    CHECKS=(
      "All 15 agents show ACTIVE lifecycle state"
      "'Put the red cube in the white box' produces correct CoT log"
      "Subgoals generated (4+ subgoals)"
      "Confidence threshold triggers approval request"
      "Failure classified correctly after injected fault"
      "World model persists across restart"
    )
    ;;
  "stage/3b-world-model-skills")
    CHECKS=(
      "All validate_stage3.py tests pass"
      "Dashboard accessible at localhost:3000"
      "Live camera feed in dashboard"
      "All 10 skills available in SkillManager"
      "Learn mode records HDF5 dataset"
      "Moving target grasped (rolling cylinder)"
    )
    ;;
  "stage/4-hardware")
    CHECKS=(
      "All validate_stage4.py tests pass on real hardware"
      "Sim→Real: each joint tracks with < 3° error"
      "Real→Sim: ADC feedback updates sim in real-time"
      "Teach mode records and replays trajectory"
      "Real pick-and-place SUCCESS on hardware"
      "No safety violations during hardware session"
    )
    ;;
  "upgrade/u1-llm-planning")
    CHECKS=(
      "All validate_upgrade_u1.py tests pass"
      "Ollama running, model loaded"
      "Complex multi-step plan generated as valid JSON"
      "Fallback to rule-based when Ollama offline"
      "VRAM freed after planning (nvidia-smi confirms)"
      "Temporal reference resolved from episodic memory"
    )
    ;;
  "upgrade/u2-advanced-perception")
    CHECKS=(
      "All validate_upgrade_u2.py tests pass"
      "SAM2 masks IoU > 0.85"
      "FoundationPose orientation error < 10°"
      "ClearGrasp corrects transparent depth"
      "Material correctly identified for 3 test objects"
      "Cup grasped by handle (affordance from SAM2 mask)"
    )
    ;;
  "upgrade/u3-infrastructure")
    CHECKS=(
      "All validate_upgrade_u3.py tests pass"
      "docker compose up starts full stack"
      "MLflow experiment logged and visible at localhost:5000"
      "Bag recording creates organized file structure"
      "Bag search returns correct results"
      "GitHub Actions CI passes on push"
    )
    ;;
  *)
    CHECKS=("Manually verify all stage requirements are met")
    ;;
esac

echo ""
for i in "${!CHECKS[@]}"; do
  echo "  $((i+1)). ${CHECKS[$i]}"
done

echo ""
read -rp "Have all checks passed? (yes/no): " CONFIRM
if [ "$CONFIRM" != "yes" ]; then
  echo "❌ Stage completion cancelled. Run validation scripts first."
  exit 1
fi

# ── Create final commit ──────────────────────────────────────────
echo ""
echo "Creating completion commit..."

# Get count of commits on this branch since branching from develop
COMMIT_COUNT=$(git log develop.."$STAGE_BRANCH" --oneline | wc -l)

git commit --allow-empty -m "release: $TAG — $STAGE_DESC

Validation: all checks passed before tagging.

Stage: $STAGE_BRANCH
Commits in stage: $COMMIT_COUNT
Timestamp: $(date -u +"%Y-%m-%dT%H:%M:%SZ")

Checklist:
$(for check in "${CHECKS[@]}"; do echo "  [x] $check"; done)"

echo "✅ Completion commit created"

# ── Tag this commit ──────────────────────────────────────────────
echo ""
echo "Creating annotated tag $TAG..."

git tag -a "$TAG" -m "ARIA $TAG — $STAGE_DESC

$(git log develop.."$STAGE_BRANCH" --oneline | head -20)

Full changelog: https://github.com/$(gh repo view --json nameWithOwner -q .nameWithOwner)/compare/develop...$STAGE_BRANCH"

echo "✅ Tag $TAG created"

# ── Push branch and tag ──────────────────────────────────────────
git push origin "$STAGE_BRANCH"
git push origin "$TAG"
echo "✅ Pushed branch and tag"

# ── Open PR to develop ───────────────────────────────────────────
echo ""
echo "Opening Pull Request to develop..."

PR_BODY="## $STAGE_DESC

### Validation Status
All stage validation checks passed before this PR was opened.

$(for check in "${CHECKS[@]}"; do echo "- [x] $check"; done)

### Changes
$(git log develop.."$STAGE_BRANCH" --oneline | head -20)

### Testing
- [ ] validate_stage*.py / validate_upgrade_u*.py: all green
- [ ] Manual review of key files
- [ ] No regressions vs previous stage

### Notes
Squash merge recommended to keep develop history clean.
Tag: \`$TAG\` applied to tip of this branch."

gh pr create \
  --title "feat: $STAGE_DESC ($TAG)" \
  --body "$PR_BODY" \
  --base develop \
  --head "$STAGE_BRANCH" \
  --label "stage-complete" \
  --label "ready-for-merge"

echo "✅ PR opened"
echo ""
echo "════════════════════════════════════════"
echo " $STAGE_DESC COMPLETE"
echo " Tag: $TAG pushed"
echo " PR: open in GitHub"
echo ""
echo " Next: review and merge PR to develop"
echo "        then merge develop to main for release"
echo "════════════════════════════════════════"
