#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
OLD_REF=""
NEW_REF=""
BUSINESS_TIP="HEAD"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --old) OLD_REF="$2"; shift 2 ;;
    --new) NEW_REF="$2"; shift 2 ;;
    --business-tip) BUSINESS_TIP="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: merge_assist.sh --old <ref> --new <ref> [--business-tip <ref>] [--repo-root DIR]"
      echo "  --business-tip  tip for business delta (default HEAD). Post-merge: use 'HEAD^1'."
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
git rev-parse --verify "${BUSINESS_TIP}^{commit}" >/dev/null 2>&1 || vs_die "invalid --business-tip: $BUSINESS_TIP"

TMPDIR_VS="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_VS"' EXIT

git diff --name-only --diff-filter=U 2>/dev/null | vs_lines_sort >"$TMPDIR_VS/conflicts" || true
git diff --name-only "${OLD_REF}..${NEW_REF}" -- t9_os | vs_lines_sort >"$TMPDIR_VS/oem"
git diff --name-only "${OLD_REF}...${BUSINESS_TIP}" -- t9_os | vs_lines_sort >"$TMPDIR_VS/biz"
comm -12 "$TMPDIR_VS/oem" "$TMPDIR_VS/biz" >"$TMPDIR_VS/intersect" || true
"$SCRIPT_DIR/list_tuya_marked.sh" --repo-root "$ROOT" | vs_lines_sort >"$TMPDIR_VS/marked" || true
comm -12 "$TMPDIR_VS/oem" "$TMPDIR_VS/marked" >"$TMPDIR_VS/marked_oem" || true

echo "## CONFLICTS"
if [[ ! -s "$TMPDIR_VS/conflicts" ]]; then
  echo "(none)"
else
  cat "$TMPDIR_VS/conflicts"
fi
echo
echo "## INTERSECT"
if [[ ! -s "$TMPDIR_VS/intersect" ]]; then
  echo "(none)"
else
  cat "$TMPDIR_VS/intersect"
fi
echo
echo "## MARKED_OEM"
if [[ ! -s "$TMPDIR_VS/marked_oem" ]]; then
  echo "(none)"
else
  cat "$TMPDIR_VS/marked_oem"
fi

if [[ -s "$TMPDIR_VS/conflicts" ]]; then
  exit 2
fi
exit 0
