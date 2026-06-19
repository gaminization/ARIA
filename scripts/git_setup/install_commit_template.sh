#!/bin/bash
cd ~/aria_ws
git config commit.template .github/commit_template.txt
echo "✅ Commit template installed"
echo "   It will appear when you run: git commit (without -m)"
