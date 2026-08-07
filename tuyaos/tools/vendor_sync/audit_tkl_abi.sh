#!/usr/bin/env bash
# Axis-B helper: OEM public include ABI changes × tuyaos_adapter references.
# Complements report_upgrade.sh (Axis-A: business customization / TUYA markers).
set -euo pipefail
export LC_ALL=C
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

REPO_ROOT=""
OLD_REF=""
NEW_REF=""
ADAPTER_DIR="tuyaos/tuyaos_adapter"
OUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --old) OLD_REF="$2"; shift 2 ;;
    --new) NEW_REF="$2"; shift 2 ;;
    --adapter-dir) ADAPTER_DIR="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage: audit_tkl_abi.sh --old <ref> --new <ref> [--repo-root DIR] [--adapter-dir PATH] [--out FILE]

Diff OEM public headers (t9_os/{ap,cp}/include) between two upstream tags,
extract removed-looking identifiers (struct fields / prototypes), and search
tuyaos_adapter for hits. Print a Markdown draft for Axis-B (TKL ABI) review.

This does NOT replace report_upgrade.sh (Axis-A). Run both after a vendor merge.
EOF
      exit 0
      ;;
    *) vs_die "unknown arg: $1" ;;
  esac
done

[[ -n "$OLD_REF" && -n "$NEW_REF" ]] || vs_die "--old and --new are required"
ROOT="$(vs_resolve_repo_root "${REPO_ROOT}")"
cd "$ROOT"
git rev-parse --verify "${OLD_REF}^{commit}" >/dev/null 2>&1 || vs_die "invalid --old: $OLD_REF"
git rev-parse --verify "${NEW_REF}^{commit}" >/dev/null 2>&1 || vs_die "invalid --new: $NEW_REF"
[[ -d "$ADAPTER_DIR" ]] || vs_die "adapter dir not found: $ADAPTER_DIR"

TMPDIR_VS="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_VS"' EXIT

INCLUDE_PATHS=(t9_os/ap/include t9_os/cp/include)

git diff --name-only "${OLD_REF}..${NEW_REF}" -- "${INCLUDE_PATHS[@]}" \
  | vs_lines_sort >"$TMPDIR_VS/changed_hdrs" || true

git diff -U0 "${OLD_REF}..${NEW_REF}" -- "${INCLUDE_PATHS[@]}" \
  >"$TMPDIR_VS/include.diff" || true

# Removed lines that look like C members / prototypes / #define names
# Examples: "qspi_wire_mode_t wire_mode;", "bk_err_t foo(...);"
awk '
  /^-[^-]/ {
    line = substr($0, 2)
    gsub(/\r/, "", line)
    # skip noise
    if (line ~ /^[[:space:]]*$/) next
    if (line ~ /^[[:space:]]*(\/\/|\/\*|\*|#include|#if|#endif|#else|#elif)/) next
    if (line ~ /^[[:space:]]*[{}]$/) next
    print line
  }
' "$TMPDIR_VS/include.diff" >"$TMPDIR_VS/removed_lines" || true

# Pull candidate identifiers (last identifier before (; or , or =)
: >"$TMPDIR_VS/symbols"
while IFS= read -r line || [[ -n "${line:-}" ]]; do
  [[ -z "${line// }" ]] && continue
  # #define NAME ...
  if [[ "$line" =~ ^[[:space:]]*#define[[:space:]]+([A-Za-z_][A-Za-z0-9_]*) ]]; then
    echo "${BASH_REMATCH[1]}" >>"$TMPDIR_VS/symbols"
    continue
  fi
  # strip trailing comments
  bare="${line%%//*}"
  bare="${bare%%/\*}"
  # function / field: ... name; or ... name( or ... name =
  if [[ "$bare" =~ ([A-Za-z_][A-Za-z0-9_]*)[[:space:]]*(\(|;|,|=|$) ]]; then
    # Prefer the last token that looks like a name before (;|=
    cand="$(echo "$bare" | sed -E 's/[[:space:]]+/ /g' | sed -E 's/[*&]//g')"
    # take last word-like token before ( or ; or ,
    token="$(echo "$cand" | sed -E 's/\(.*//' | sed -E 's/;.*//' | sed -E 's/,.*//' | sed -E 's/=.*//' | awk '{print $NF}')"
    # filter obvious types / keywords
    case "$token" in
      ""|typedef|struct|enum|union|const|volatile|static|inline|extern|void|int|char|long|short|unsigned|signed|return|if|else|for|while|do|switch|case|break|continue|sizeof|true|false)
        continue
        ;;
      *_t|*_E|*_T)
        # type names alone are weak signals; still keep if removed as field type+name pair handled below
        ;;
    esac
    if [[ "$token" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
      echo "$token" >>"$TMPDIR_VS/symbols"
    fi
  fi
done <"$TMPDIR_VS/removed_lines"

# Also explicitly pick "type name;" field pattern → name
grep -E '[[:alnum:]_]+[[:space:]]+[*]?[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*;' "$TMPDIR_VS/removed_lines" \
  | sed -E 's/\/\*.*\*\///g; s/\/\/.*//' \
  | sed -E 's/[[:space:]]+/ /g' \
  | sed -E 's/^[[:space:]]*//; s/[[:space:]]*;[[:space:]]*$//' \
  | awk '{print $NF}' \
  | sed -E 's/^\*+//' \
  >>"$TMPDIR_VS/symbols" || true

# Normalize: drop leading * from pointers (e.g. *out_buffer → out_buffer)
sed -E 's/^\*+//' "$TMPDIR_VS/symbols" | vs_lines_sort >"$TMPDIR_VS/symbols_u"
# drop ultra-common false positives
grep -Ev '^(uint8_t|uint16_t|uint32_t|uint64_t|int8_t|int16_t|int32_t|int64_t|size_t|bool|BOOL_T|UINT8_T|UINT16_T|UINT32_T|VOID_T|CHAR_T|UCHAR_T)$' \
  "$TMPDIR_VS/symbols_u" >"$TMPDIR_VS/symbols_f" || true

vs_search_adapter() {
  local sym="$1"
  if command -v rg >/dev/null 2>&1; then
    rg -n --glob 'tkl_*.{c,h}' --glob '*.c' --glob '*.h' -F "$sym" "$ADAPTER_DIR" 2>/dev/null | head -20 || true
  else
    # Fallback when ripgrep is not on PATH (common in plain SSH shells)
    grep -RIn --include='tkl_*.c' --include='tkl_*.h' --include='*.c' --include='*.h' -F "$sym" "$ADAPTER_DIR" 2>/dev/null | head -20 || true
  fi
}

SEARCH_TOOL="grep"
command -v rg >/dev/null 2>&1 && SEARCH_TOOL="rg"

: >"$TMPDIR_VS/hits"
HIT_N=0
while IFS= read -r sym || [[ -n "${sym:-}" ]]; do
  [[ -z "$sym" ]] && continue
  # skip 1–2 char tokens / non-identifiers
  [[ ${#sym} -lt 3 ]] && continue
  [[ "$sym" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || continue
  matches="$(vs_search_adapter "$sym")"
  if [[ -n "$matches" ]]; then
    {
      echo "### \`$sym\`"
      echo
      echo '```'
      echo "$matches"
      echo '```'
      echo
    } >>"$TMPDIR_VS/hits"
    HIT_N=$((HIT_N + 1))
  fi
done <"$TMPDIR_VS/symbols_f"

HDR_N="$(grep -c . "$TMPDIR_VS/changed_hdrs" 2>/dev/null || echo 0)"
SYM_N="$(grep -c . "$TMPDIR_VS/symbols_f" 2>/dev/null || echo 0)"
[[ -z "$HDR_N" || "$HDR_N" == "" ]] && HDR_N=0
[[ -z "$SYM_N" || "$SYM_N" == "" ]] && SYM_N=0

STAT="$(git diff --stat "${OLD_REF}..${NEW_REF}" -- "${INCLUDE_PATHS[@]}" || true)"

emit() {
  cat <<EOF
# TKL ABI audit (Axis-B)

- Old: ${OLD_REF}
- New: ${NEW_REF}
- Adapter: ${ADAPTER_DIR}
- Changed public headers: ${HDR_N}
- Candidate removed symbols: ${SYM_N}
- Symbols with hits in adapter: ${HIT_N}
- Search tool: ${SEARCH_TOOL}

> Auto-generated draft. False positives are expected — triage MUST / LIKELY / NO manually.
> Axis-A (customization) still comes from \`report_upgrade.sh\` / INTERSECT / TUYA markers.
> If hits are 0 but candidates >> 0, check Search tool / PATH; still triage **Include diffstat** by hand (e.g. \`hal_qspi_types.h\`).

## Include diffstat

\`\`\`
${STAT}
\`\`\`

## Changed header paths

$(if [[ ! -s "$TMPDIR_VS/changed_hdrs" ]]; then echo '*(none)*'; else cat "$TMPDIR_VS/changed_hdrs"; fi)

## Adapter hits (removed OEM identifiers)

$(if [[ ! -s "$TMPDIR_VS/hits" ]]; then echo '*(no automatic hits — still review struct renames manually)*'; else cat "$TMPDIR_VS/hits"; fi)

## Manual triage checklist

- [ ] Struct fields deleted/renamed (compile break) → MUST change matching \`tkl_*.c\`
- [ ] Prototype removed/changed → MUST or LIKELY
- [ ] Only behavior in \`.c\` under middleware (header unchanged) → regression, may skip code change
- [ ] Cross-check high-touch TKL: wifi / gpu / venc / vdec / uvc / flash / ota / ipc / qspi / camera
EOF
}

if [[ -n "$OUT" ]]; then
  # Quoted "~/path" does not expand in the caller's shell; expand leading ~ here.
  # Use case (not [[ == ~/* ]]) so RHS tilde/glob is not expanded by bash.
  case "$OUT" in
    '~'|'~/'*) OUT="${HOME}${OUT#\~}" ;;
  esac
  out_dir="$(dirname -- "$OUT")"
  mkdir -p -- "$out_dir" || vs_die "cannot create output dir: $out_dir"
  emit >"$OUT"
  echo "wrote $OUT" >&2
else
  emit
fi
