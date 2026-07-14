#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: list_tuya_marked.sh [--repo-root DIR]"
      exit 0
      ;;
    *) vs_die "unknown arg: $1" ;;
  esac
done

vs_need_cmd rg
ROOT="$(vs_resolve_repo_root "${REPO_ROOT}")"
cd "$ROOT"

rg -l --glob '!**/bk_tflite_micro/**' \
  -e 'Modified by TUYA' -e 'Modified by Tuya' -e 'Added by TUYA' \
  t9_os 2>/dev/null | LC_ALL=C sort -u || true
