#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
UPSTREAM_REF=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --upstream) UPSTREAM_REF="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: list_business_delta.sh --upstream <ref> [--repo-root DIR]"
      echo "Prints: git diff --name-status <upstream>...HEAD -- t9_os"
      exit 0
      ;;
    *) vs_die "unknown arg: $1" ;;
  esac
done

[[ -n "$UPSTREAM_REF" ]] || vs_die "--upstream is required"
ROOT="$(vs_resolve_repo_root "${REPO_ROOT}")"
cd "$ROOT"
git rev-parse --verify "$UPSTREAM_REF^{commit}" >/dev/null 2>&1 \
  || vs_die "invalid upstream ref: $UPSTREAM_REF"

git diff --name-status "${UPSTREAM_REF}...HEAD" -- t9_os
