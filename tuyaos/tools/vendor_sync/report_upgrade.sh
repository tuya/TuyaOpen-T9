#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
OLD_REF=""
NEW_REF=""
OUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --old) OLD_REF="$2"; shift 2 ;;
    --new) NEW_REF="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: report_upgrade.sh --old <ref> --new <ref> [--repo-root DIR] [--out FILE]"
      exit 0
      ;;
    *) vs_die "unknown arg: $1" ;;
  esac
done

[[ -n "$OLD_REF" && -n "$NEW_REF" ]] || vs_die "--old and --new are required"
ROOT="$(vs_resolve_repo_root "${REPO_ROOT}")"
cd "$ROOT"
git rev-parse --verify "$OLD_REF^{commit}" >/dev/null 2>&1 || vs_die "invalid --old: $OLD_REF"
git rev-parse --verify "$NEW_REF^{commit}" >/dev/null 2>&1 || vs_die "invalid --new: $NEW_REF"

HEAD_SHA="$(git rev-parse --short HEAD)"

TMPDIR_VS="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_VS"' EXIT

git diff --name-only "${OLD_REF}..${NEW_REF}" -- t9_os | vs_lines_sort >"$TMPDIR_VS/oem"
git diff --name-only "${OLD_REF}...HEAD" -- t9_os | vs_lines_sort >"$TMPDIR_VS/biz"
comm -12 "$TMPDIR_VS/oem" "$TMPDIR_VS/biz" >"$TMPDIR_VS/intersect" || true

"$SCRIPT_DIR/list_tuya_marked.sh" --repo-root "$ROOT" | vs_lines_sort >"$TMPDIR_VS/marked" || true
comm -12 "$TMPDIR_VS/oem" "$TMPDIR_VS/marked" >"$TMPDIR_VS/marked_oem" || true

git diff --name-status "${OLD_REF}...HEAD" -- t9_os | awk '$1=="A"{print $2}' | vs_lines_sort >"$TMPDIR_VS/added_biz"
git diff --name-status "${OLD_REF}..${NEW_REF}" -- t9_os | awk '$1=="A"{print $2}' | vs_lines_sort >"$TMPDIR_VS/added_oem"
comm -12 "$TMPDIR_VS/added_biz" "$TMPDIR_VS/added_oem" >"$TMPDIR_VS/collision" || true

DIFFSTAT="$(git diff --stat "${OLD_REF}..${NEW_REF}" -- t9_os)"

count_file() {
  local f="$1"
  if [[ ! -s "$f" ]]; then
    echo 0
  else
    grep -c . "$f"
  fi
}

section_file() {
  local f="$1"
  if [[ ! -s "$f" ]]; then
    echo "*(none)*"
  else
    cat "$f"
  fi
}

emit() {
  cat <<EOF
# T9 upstream upgrade report

- Old: ${OLD_REF}
- New: ${NEW_REF}
- HEAD: ${HEAD_SHA}

## OEM changed files ($(count_file "$TMPDIR_VS/oem"))

$(section_file "$TMPDIR_VS/oem")

## Business delta vs old ($(count_file "$TMPDIR_VS/biz"))

$(section_file "$TMPDIR_VS/biz")

## Intersection business ∩ OEM (PRIMARY REVIEW QUEUE) ($(count_file "$TMPDIR_VS/intersect"))

$(section_file "$TMPDIR_VS/intersect")

## TUYA-marked ∩ OEM ($(count_file "$TMPDIR_VS/marked_oem"))

$(section_file "$TMPDIR_VS/marked_oem")

## Business-only added files ($(count_file "$TMPDIR_VS/added_biz"))

$(section_file "$TMPDIR_VS/added_biz")

## Path collisions (business added ∩ OEM added) ($(count_file "$TMPDIR_VS/collision"))

$(section_file "$TMPDIR_VS/collision")

## Diffstat (OEM old..new)

\`\`\`
${DIFFSTAT}
\`\`\`
EOF
}

if [[ -n "$OUT" ]]; then
  emit >"$OUT"
  echo "report_upgrade: wrote $OUT"
else
  emit
fi
