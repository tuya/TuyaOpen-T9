#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
SRC=""
COMMIT=0
MSG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --src) SRC="$2"; shift 2 ;;
    --commit) COMMIT=1; shift ;;
    --message) MSG="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: import_pristine.sh --src OEM_DIR [--repo-root DIR] [--commit] [--message MSG]"
      echo "Rsync OEM tree into t9_os/, excluding .git and common noise. Does not push or tag."
      exit 0
      ;;
    *) vs_die "unknown arg: $1" ;;
  esac
done

[[ -n "$SRC" && -d "$SRC" ]] || vs_die "--src directory required"
vs_need_cmd rsync
ROOT="$(vs_resolve_repo_root "${REPO_ROOT}")"
cd "$ROOT"

mkdir -p t9_os
rsync -a --delete \
  --exclude '.git/' \
  --exclude '.github/' \
  --exclude '.vscode/' \
  --exclude '.idea/' \
  "$SRC"/ t9_os/

[[ ! -e t9_os/.git ]] || vs_die "t9_os/.git still present after rsync; abort"

if [[ "$COMMIT" -eq 1 ]]; then
  git add -A t9_os
  git commit -m "${MSG:-Import pristine t9_os}"
fi

echo "import_pristine: synced $SRC -> $ROOT/t9_os"
