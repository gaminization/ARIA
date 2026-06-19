#!/bin/bash
# Applies GitHub branch protection rules via GitHub CLI.
# Run after init_repository.sh.

set -euo pipefail
GITHUB_USER=$1
REPO_NAME=$2
REPO="$GITHUB_USER/$REPO_NAME"

apply_protection() {
  local branch=$1
  local require_pr=$2          # true | false
  local required_checks=$3     # JSON array of check names
  local require_review=$4      # true | false
  local allow_force_push=$5    # true | false

  gh api \
    --method PUT \
    -H "Accept: application/vnd.github+json" \
    "/repos/$REPO/branches/$branch/protection" \
    -f required_status_checks="{
      \"strict\": true,
      \"contexts\": $required_checks
    }" \
    -F enforce_admins=false \
    -f required_pull_request_reviews="{
      \"required_approving_review_count\": $([ "$require_review" = "true" ] && echo 1 || echo 0),
      \"dismiss_stale_reviews\": true,
      \"require_code_owner_reviews\": false
    }" \
    -F allow_force_pushes="$allow_force_push" \
    -F allow_deletions=false \
    -F block_creations=false \
    2>/dev/null && echo "  ✅ Protected: $branch" \
              || echo "  ⚠️  Could not protect: $branch (may need GitHub Pro for private repos)"
}

echo "Applying branch protection rules..."

# main: strictest — PR required, all checks must pass, 1 reviewer
apply_protection "main" \
  "true" \
  '["build","unit_tests"]' \
  "true" \
  "false"

# develop: PR required, build must pass, no reviewer required
apply_protection "develop" \
  "true" \
  '["build","unit_tests"]' \
  "false" \
  "false"

# Note: stage/* and upgrade/* branches are NOT protected
# Direct pushes allowed — developer works solo
# They go through PR when merging INTO develop

echo "✅ Branch protection applied"
echo ""
echo "Rules summary:"
echo "  main:    PR required, 1 reviewer, all CI checks"
echo "  develop: PR required, CI checks, no reviewer"
echo "  stage/*: Direct push allowed (solo development)"
echo "  upgrade/*: Direct push allowed (solo development)"
