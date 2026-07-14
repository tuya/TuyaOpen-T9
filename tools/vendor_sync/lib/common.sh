#!/usr/bin/env bash
# Shared helpers for vendor_sync tools.
set -euo pipefail

vs_die() {
  echo "vendor_sync: error: $*" >&2
  exit 1
}

vs_need_cmd() {
  command -v "$1" >/dev/null 2>&1 || vs_die "missing required command: $1"
}

# Resolve vendor/T9 repo root: --repo-root, or walk up looking for t9_os/ + .git
vs_resolve_repo_root() {
  local explicit="${1:-}"
  if [[ -n "$explicit" ]]; then
    [[ -d "$explicit/t9_os" && -d "$explicit/.git" ]] || vs_die "not a vendor/T9 root: $explicit"
    (cd "$explicit" && pwd)
    return
  fi
  local d
  d="$(pwd)"
  while true; do
    if [[ -d "$d/t9_os" && -d "$d/.git" ]]; then
      echo "$d"
      return
    fi
    [[ "$d" == "/" ]] && vs_die "cannot find vendor/T9 root (need t9_os/ and .git); use --repo-root"
    d="$(dirname "$d")"
  done
}

vs_need_cmd git
