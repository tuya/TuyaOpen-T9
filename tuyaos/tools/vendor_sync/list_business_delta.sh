#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
UPSTREAM_REF=""
BUSINESS_TIP="HEAD"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --upstream) UPSTREAM_REF="$2"; shift 2 ;;
    --business-tip) BUSINESS_TIP="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: list_business_delta.sh --upstream <ref> [--business-tip <ref>] [--repo-root DIR]"
      echo "Prints: git diff --name-status <upstream>...<business-tip> -- t9_os"
      echo "  --business-tip  default HEAD; post-merge Axis A: use 'HEAD^1'."
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
git rev-parse --verify "${BUSINESS_TIP}^{commit}" >/dev/null 2>&1 \
  || vs_die "invalid --business-tip: $BUSINESS_TIP"

git diff --name-status "${UPSTREAM_REF}...${BUSINESS_TIP}" -- t9_os
