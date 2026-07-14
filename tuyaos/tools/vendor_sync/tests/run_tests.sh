#!/usr/bin/env bash
set -euo pipefail
TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYNC_DIR="$(cd "$TESTS_DIR/.." && pwd)"
FIX="$(mktemp -d)"
trap 'rm -rf "$FIX"' EXIT

assert_eq() {
  local name="$1" got="$2" want="$3"
  if [[ "$got" != "$want" ]]; then
    echo "FAIL: $name"$'\n'"got:"$'\n'"$got"$'\n'"want:"$'\n'"$want" >&2
    exit 1
  fi
  echo "PASS: $name"
}

assert_contains() {
  local name="$1" hay="$2" needle="$3"
  if ! grep -Fq -- "$needle" <<<"$hay"; then
    echo "FAIL: $name missing: $needle"$'\n'"hay:"$'\n'"$hay" >&2
    exit 1
  fi
  echo "PASS: $name contains ${needle}"
}

# ---------- build fixture repo ----------
REPO="$FIX/repo"
mkdir -p "$REPO/t9_os/driver"

cd "$REPO"
git init -q
git config user.email "test@example.com"
git config user.name "test"

# upstream/bk-1.0 pristine
printf 'shared v1\n' >t9_os/driver/shared.c
printf 'base only\n' >t9_os/driver/a.c
git add t9_os
git commit -qm "Import pristine t9_os 1.0"
git tag upstream/bk-1.0

# upstream/bk-1.1 on side branch (modify shared + add oem_only)
git checkout -qb upstream_bk
printf 'shared v2\n' >t9_os/driver/shared.c
printf 'oem new\n' >t9_os/driver/oem_only.c
git add -A t9_os
git commit -qm "Import pristine t9_os 1.1"
git tag upstream/bk-1.1

# back to primary branch at 1.0
if git show-ref --verify --quiet refs/heads/master; then
  git checkout -q master
elif git show-ref --verify --quiet refs/heads/main; then
  git checkout -q main
else
  git checkout -q upstream/bk-1.0
  git checkout -qb master
fi

# Business deltas on top of 1.0
printf 'shared biz\n' >t9_os/driver/shared.c
printf '// Modified by TUYA Start\nmarked body\n// Modified by TUYA End\n' >t9_os/driver/b_marked.c
printf 'biz new\n// Added by TUYA\n' >t9_os/driver/biz_only.c
git add -A t9_os
git commit -qm "biz customizations"

# ---------- list_tuya_marked ----------
MARKED="$("$SYNC_DIR/list_tuya_marked.sh" --repo-root "$REPO")"
assert_eq "list_tuya_marked" "$MARKED" $'t9_os/driver/b_marked.c\nt9_os/driver/biz_only.c'

# ---------- list_business_delta ----------
DELTA="$("$SYNC_DIR/list_business_delta.sh" --repo-root "$REPO" --upstream upstream/bk-1.0)"
assert_contains "list_business_delta M shared" "$DELTA" $'M\tt9_os/driver/shared.c'
assert_contains "list_business_delta A biz_only" "$DELTA" $'A\tt9_os/driver/biz_only.c'
assert_contains "list_business_delta A b_marked" "$DELTA" $'A\tt9_os/driver/b_marked.c'

# ---------- import_pristine excludes .git ----------
OEM_GITTY="$FIX/oem_gitty"
mkdir -p "$OEM_GITTY/driver" "$OEM_GITTY/.git"
printf 'clean oem\n' >"$OEM_GITTY/driver/x.c"
printf 'gitdir\n' >"$OEM_GITTY/.git/config"
IMPORT_REPO="$FIX/import_repo"
mkdir -p "$IMPORT_REPO/t9_os"
cd "$IMPORT_REPO"
git init -q
git config user.email "test@example.com"
git config user.name "test"
printf 'placeholder\n' >t9_os/keep.txt
git add t9_os && git commit -qm init
"$SYNC_DIR/import_pristine.sh" --repo-root "$IMPORT_REPO" --src "$OEM_GITTY" >/dev/null
[[ ! -e "$IMPORT_REPO/t9_os/.git" ]] || { echo "FAIL: nested .git remained"; exit 1; }
[[ -f "$IMPORT_REPO/t9_os/driver/x.c" ]] || { echo "FAIL: rsync missing file"; exit 1; }
echo "PASS: import_pristine excludes .git"

# ---------- report_upgrade ----------
cd "$REPO"
REPORT="$("$SYNC_DIR/report_upgrade.sh" --repo-root "$REPO" --old upstream/bk-1.0 --new upstream/bk-1.1)"
assert_contains "report has intersection header" "$REPORT" "PRIMARY REVIEW QUEUE"
INTERSECT_SECTION="$(awk "/PRIMARY REVIEW QUEUE/,/TUYA-marked/" <<<"$REPORT" | head -n -1)"
assert_contains "report intersection shared" "$INTERSECT_SECTION" "t9_os/driver/shared.c"
assert_contains "report biz added" "$REPORT" "t9_os/driver/biz_only.c"
assert_contains "report oem added" "$REPORT" "t9_os/driver/oem_only.c"
OEM_SECTION="$(awk "/^## OEM changed/,/^## Business delta/" <<<"$REPORT")"
assert_contains "oem section has shared" "$OEM_SECTION" "t9_os/driver/shared.c"
echo "PASS: report_upgrade intersection and lists"

# ---------- merge_assist ----------
set +e
ASSIST_OUT="$("$SYNC_DIR/merge_assist.sh" --repo-root "$REPO" --old upstream/bk-1.0 --new upstream/bk-1.1)"
ASSIST_RC=$?
set -e
[[ "$ASSIST_RC" -eq 0 ]] || { echo "FAIL: merge_assist rc=$ASSIST_RC"; echo "$ASSIST_OUT"; exit 1; }
assert_contains "merge_assist intersect" "$ASSIST_OUT" "t9_os/driver/shared.c"
echo "PASS: merge_assist exit 0 with intersect"

echo "All vendor_sync tests passed."
