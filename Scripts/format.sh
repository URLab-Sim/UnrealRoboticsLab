#!/usr/bin/env bash
# Format URLab C++ with the repo .clang-format (Epic's canonical UE style).
#
# Usage:
#   Scripts/format.sh            # format Source/ in place
#   Scripts/format.sh --check    # exit 1 if anything is unformatted (CI)
#
# Codegen-managed files are also formatted here, but the codegen emits the
# same formatting (see _clang_format_content in generate_ue_components.py), so
# running this after a regen is a no-op on generated files.
#
# Requires clang-format 19.x. Set $CLANG_FORMAT, or have clang-format(-19) on PATH.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

resolve_cf() {
    if [[ -n "${CLANG_FORMAT:-}" && -x "${CLANG_FORMAT}" ]]; then echo "${CLANG_FORMAT}"; return; fi
    if command -v clang-format-19 >/dev/null 2>&1; then command -v clang-format-19; return; fi
    if command -v clang-format    >/dev/null 2>&1; then command -v clang-format;    return; fi
    echo "clang-format not found. Set \$CLANG_FORMAT or install clang-format 19.x." >&2
    exit 1
}
CF="$(resolve_cf)"

mapfile -t FILES < <(find "${ROOT}/Source" -type f \( -name '*.h' -o -name '*.cpp' \) \
    -not -path '*/Intermediate/*' -not -path '*/Binaries/*' | sort)

echo "clang-format: ${CF}"
echo "files: ${#FILES[@]}"

if [[ "${1:-}" == "--check" ]]; then
    bad=0
    for f in "${FILES[@]}"; do
        if ! "${CF}" --style=file --dry-run --Werror "${f}" 2>/dev/null; then
            echo "  UNFORMATTED: ${f}"; bad=$((bad + 1))
        fi
    done
    if [[ ${bad} -gt 0 ]]; then echo "UNFORMATTED files: ${bad}" >&2; exit 1; fi
    echo "All formatted."; exit 0
fi

for f in "${FILES[@]}"; do "${CF}" -i --style=file "${f}"; done
echo "Formatted ${#FILES[@]} files."
