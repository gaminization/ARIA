#!/bin/bash
# ARIA Repository Initialization Script
# Run ONCE from inside ~/aria_ws after completing each stage.
# Usage: ./init_repository.sh --github-user <username> --repo aria
# Prerequisites: git, gh (GitHub CLI) installed and authenticated

set -euo pipefail

# ── Parse arguments ──────────────────────────────────────────────
GITHUB_USER=""
REPO_NAME="aria"
VISIBILITY="private"      # private until Stage 4 passes, then public

while [[ $# -gt 0 ]]; do
  case $1 in
    --github-user) GITHUB_USER="$2"; shift 2 ;;
    --repo)        REPO_NAME="$2";   shift 2 ;;
    --public)      VISIBILITY="public"; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

[[ -z "$GITHUB_USER" ]] && { echo "Error: --github-user required"; exit 1; }

echo "════════════════════════════════════════"
echo " ARIA Repository Setup"
echo " User: $GITHUB_USER"
echo " Repo: $REPO_NAME ($VISIBILITY)"
echo "════════════════════════════════════════"

# ── Step 1: Local git init ───────────────────────────────────────
cd ~/aria_ws

if [ ! -d .git ]; then
  git init
  echo "✅ Git initialized"
else
  echo "ℹ️  Git already initialized, skipping"
fi

# ── Step 2: Set git identity if not set ─────────────────────────
if [ -z "$(git config user.email)" ]; then
  read -rp "Git email: " GIT_EMAIL
  read -rp "Git name:  " GIT_NAME
  git config user.email "$GIT_EMAIL"
  git config user.name  "$GIT_NAME"
fi

# ── Step 3: Core git config for this repo ───────────────────────
git config core.autocrlf false
git config core.eol lf
git config pull.rebase false
git config branch.autosetuprebase never
git config merge.ff false        # always create merge commits
git config push.default current

# ── Step 4: Install pre-commit hooks ────────────────────────────
echo "Installing pre-commit hooks..."
pip install pre-commit --quiet
pre-commit install
pre-commit install --hook-type commit-msg
echo "✅ Pre-commit hooks installed"

# ── Step 5: Configure Git LFS for large files ───────────────────
git lfs install
git lfs track "*.pt"       # PyTorch model weights
git lfs track "*.pth"      # PyTorch checkpoints
git lfs track "*.onnx"     # ONNX models
git lfs track "*.hdf5"     # LeRobot datasets
git lfs track "*.db3"      # ROS2 bags
git lfs track "*.stl"      # mesh files
git lfs track "*.obj"      # mesh files
git lfs track "*.dae"      # mesh files
git lfs track "*.png"      # reference images (large batches)
git add .gitattributes
echo "✅ Git LFS configured"

# ── Step 6: Initial commit on main ──────────────────────────────
cp scripts/git_setup/templates/.gitignore .gitignore
git add .gitignore .gitattributes .github/ README.md
git commit -m "chore: initialize ARIA repository

- Add .gitignore for ROS2/Python/ML stack
- Configure Git LFS for model weights and datasets
- Add GitHub Actions CI workflow
- Add PR and issue templates
- Add pre-commit hook configuration

Co-authored-by: ARIA Setup Script <setup@aria.local>"

# ── Step 7: Create remote on GitHub ─────────────────────────────
echo "Creating GitHub repository..."
gh repo create "$GITHUB_USER/$REPO_NAME" \
  --"$VISIBILITY" \
  --description "ARIA: Adaptive Robotic Intelligence Architecture — 5-DoF Arm AI System" \
  --homepage "" \
  --add-readme=false

git remote add origin "https://github.com/$GITHUB_USER/$REPO_NAME.git"
git branch -M main
git push -u origin main
echo "✅ Remote created: https://github.com/$GITHUB_USER/$REPO_NAME"

# ── Step 8: Create all branches ─────────────────────────────────
echo ""
echo "Creating branch structure..."

create_branch() {
  local branch=$1
  local from=${2:-main}
  git checkout "$from"
  git checkout -b "$branch"
  git push -u origin "$branch"
  echo "  ✅ $branch"
}

# Integration branch
create_branch "develop" "main"

# Stage branches (from develop)
create_branch "stage/1-foundation"          "develop"
create_branch "stage/2-ik-perception"       "develop"
create_branch "stage/3a-agents-planning"    "develop"
create_branch "stage/3b-world-model-skills" "develop"
create_branch "stage/4-hardware"            "develop"

# Upgrade branches (from develop)
create_branch "upgrade/u1-llm-planning"          "develop"
create_branch "upgrade/u2-advanced-perception"   "develop"
create_branch "upgrade/u3-infrastructure"        "develop"

# Return to develop as default working branch
git checkout develop
echo ""
echo "✅ All branches created"

# ── Step 9: Apply branch protection rules via GitHub CLI ────────
echo ""
echo "Applying branch protection rules..."
bash scripts/git_setup/apply_branch_protection.sh "$GITHUB_USER" "$REPO_NAME"

# ── Step 10: Create GitHub labels ───────────────────────────────
echo ""
echo "Creating GitHub labels..."
bash scripts/git_setup/create_labels.sh "$GITHUB_USER" "$REPO_NAME"

# ── Step 11: Create GitHub milestones ───────────────────────────
echo ""
echo "Creating milestones..."
bash scripts/git_setup/create_milestones.sh "$GITHUB_USER" "$REPO_NAME"

echo ""
echo "════════════════════════════════════════"
echo " ARIA Repository Ready"
echo " URL: https://github.com/$GITHUB_USER/$REPO_NAME"
echo " Default branch: develop"
echo ""
echo " Next steps:"
echo "   1. Work on stage/1-foundation"
echo "   2. When stage 1 complete:"
echo "      ./scripts/git_setup/complete_stage.sh stage/1 v0.1.0"
echo "════════════════════════════════════════"
