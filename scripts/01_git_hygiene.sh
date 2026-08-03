#!/usr/bin/env bash
# 01_git_hygiene.sh — run once, before the first commit.
# Suppresses the Co-Authored-By trailer on commits and PRs.
set -euo pipefail
mkdir -p .claude
cat > .claude/settings.json <<'JSON'
{
  "attribution": {
    "commit": "",
    "pr": ""
  }
}
JSON
echo "wrote .claude/settings.json"
cat .claude/settings.json
