#!/bin/bash
# Merges develop into main and creates a GitHub release.
# Run after a stage PR has been merged to develop.
# Usage: ./release_to_main.sh v0.1.0

set -euo pipefail
TAG=$1

RELEASE_NOTES_MAP=(
  "v0.1.0:Gazebo simulation with full manual joint control. All 6 servos controllable via keyboard, GUI slider, and ROS2 service. Stage 1 complete."
  "v0.2.0:IK solver benchmark system (analytical/ikpy/RTB/neural). YOLO v8 detection. Depth-Anything v2 + geometric hybrid coordinate system. First autonomous grasp."
  "v0.3.0:All 15 AI agents operational. NLP task decomposition with chain-of-thought logging. Confidence-based decisions. Failure classification and recovery."
  "v0.4.0:Persistent world model with SQLite. 10 predefined skills. LeRobot/OpenVLA interface. FastAPI + React dashboard. Learn mode teleoperation."
  "v1.0.0:First real hardware deployment. ESP32 firmware. Bidirectional servo sync (sim↔real). Teach mode. Hardware bringup wizard. Production-ready."
  "v1.1.0:Local LLM planning via Ollama (Llama 3.1 8B). ReAct function calling. Tree-of-Thought for complex tasks. Episodic cross-session memory."
  "v1.2.0:FoundationPose 6D object pose. SAM2 pixel-accurate masks. ClearGrasp transparent object depth. Material recognition with grasp adaptation."
  "v1.3.0:Full Docker containerization. GitHub Actions CI/CD. MLflow experiment tracking. Automatic ROS2 bag recording. Central object model database."
)

# Find release notes for this tag
NOTES=""
for entry in "${RELEASE_NOTES_MAP[@]}"; do
  if [[ "$entry" == "$TAG:"* ]]; then
    NOTES="${entry#*:}"
    break
  fi
done

echo "════════════════════════════════════════"
echo " Releasing $TAG to main"
echo "════════════════════════════════════════"

# Verify we're on develop and it's clean
git checkout develop
git pull origin develop

if ! git diff-index --quiet HEAD --; then
  echo "❌ develop has uncommitted changes"
  exit 1
fi

# Verify tag exists on develop
TAG_COMMIT=$(git rev-list -n 1 "$TAG" 2>/dev/null || echo "none")
if [ "$TAG_COMMIT" = "none" ]; then
  echo "❌ Tag $TAG not found. Run complete_stage.sh first."
  exit 1
fi

# Merge develop → main
git checkout main
git pull origin main
git merge develop \
  --no-ff \
  -m "release: merge $TAG to main

$NOTES

Released: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"

git push origin main
echo "✅ develop merged to main"

# Create GitHub release from existing tag
gh release create "$TAG" \
  --title "ARIA $TAG" \
  --notes "## What's in this release

$NOTES

## Validation
All stage validation scripts passed before release.

## Upgrade path
\`\`\`bash
git checkout $TAG
colcon build --symlink-install
\`\`\`

## Full Changelog
See [CHANGELOG.md](CHANGELOG.md) for detailed changes." \
  --latest

echo "✅ GitHub release created: $TAG"
echo ""
echo "Release URL:"
gh release view "$TAG" --json url -q .url

# Return to develop
git checkout develop
echo ""
echo "✅ Back on develop. Release $TAG complete."
