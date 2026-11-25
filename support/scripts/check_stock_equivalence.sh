#!/usr/bin/env bash
# Checks that this tree, built in stock mode (no USE_INDIFLIGHT, no indiflight
# feature flags), produces the same firmware as an upstream Betaflight tag.
#
#   support/scripts/check_stock_equivalence.sh [TAG] [CONFIG]
#     TAG     upstream tag to compare against (default: 2026.6.2)
#     CONFIG  board config (default: MATEKH743)
#
# Env: STOCK_DIR   existing build tree of TAG to reuse (default: a worktree
#                  in $TMPDIR, created and built on first use)
#      MAKE_ARGS   extra make arguments for both builds (e.g. GCC_REQUIRED_VERSION=13.2.1)
#
# Compares `nm -S` (symbol size, type, name; addresses dropped) and the
# disassembly with addresses stripped. Prints "identical" or the differences.
# Re-run after every upstream merge.
set -euo pipefail

TAG=${1:-2026.6.2}
CONFIG=${2:-MATEKH743}
ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
TMP=${TMPDIR:-/tmp}
STOCK_DIR=${STOCK_DIR:-$TMP/bf-stock-$TAG}
MAKE_ARGS=${MAKE_ARGS:-}
NM=arm-none-eabi-nm
OBJDUMP=arm-none-eabi-objdump

if [ ! -d "$STOCK_DIR" ]; then
    git -C "$ROOT" worktree add --detach "$STOCK_DIR" "$TAG" >/dev/null
    git -C "$STOCK_DIR" submodule update --init src/config >/dev/null
fi

echo "building stock $TAG in $STOCK_DIR" >&2
make -C "$STOCK_DIR" CONFIG="$CONFIG" $MAKE_ARGS clean >/dev/null
make -C "$STOCK_DIR" CONFIG="$CONFIG" $MAKE_ARGS -j"$(nproc)" >/dev/null
echo "building port (stock mode) in $ROOT" >&2
# the revision string is "norevision" in a dirty tree, which shifts the layout
[ -z "$(git -C "$ROOT" status --porcelain -uno)" ] || echo "warning: $ROOT has uncommitted changes, expect a revision string difference" >&2
make -C "$ROOT" CONFIG="$CONFIG" OBJECT_DIR=obj/upf $MAKE_ARGS clean >/dev/null
make -C "$ROOT" CONFIG="$CONFIG" OBJECT_DIR=obj/upf $MAKE_ARGS -j"$(nproc)" >/dev/null

stockElf=$(ls "$STOCK_DIR"/obj/main/betaflight_*"$CONFIG".elf)
portElf=$(ls "$ROOT"/obj/upf/betaflight_*"$CONFIG".elf)

out=$(mktemp -d)
symbols() { $NM -S "$1" | awk 'NF == 4 { print $2, $3, $4 }' | sort; }
disasm() {
    # strip addresses and raw bytes, keep the instructions; drop the file name line.
    # Linker veneers (long-branch stubs) are dropped: their order is not stable
    # between builds, their names and sizes are compared with the symbols.
    # Lines with the build date/time and git revisions are dropped as well.
    $OBJDUMP -d --no-show-raw-insn "$1" \
        | sed -E 's/^ *[0-9a-f]+:\t//; s/\<(0x)?[0-9a-f]{5,8}\>/ADDR/g; s/\<[0-9a-f]{1,8} </ADDR </g' | tail -n +4 \
        | awk '/_veneer>:$/ { skip = 1 } skip && /^$/ { skip = 0; next } !skip' \
        | grep -v -E "[0-9]{2}:[0-9]{2}:[0-9]{2}|$REVS"
}
# git revisions of both trees (they end up in the firmware as strings)
REVS="$(git -C "$STOCK_DIR" rev-parse --short=9 HEAD)|$(git -C "$ROOT" rev-parse --short=9 HEAD)"
symbols "$stockElf" > "$out/stock.sym"
symbols "$portElf" > "$out/port.sym"
disasm "$stockElf" > "$out/stock.dis"
disasm "$portElf" > "$out/port.dis"

status=0
if diff -q "$out/stock.sym" "$out/port.sym" >/dev/null; then
    echo "symbols: identical"
else
    echo "symbols: differ"; diff "$out/stock.sym" "$out/port.sym" || true; status=1
fi
if diff -q "$out/stock.dis" "$out/port.dis" >/dev/null; then
    echo "disassembly: identical"
else
    n=$(diff "$out/stock.dis" "$out/port.dis" | grep -c '^[<>]' || true)
    echo "disassembly: $n differing lines (see $out/*.dis)"; status=1
fi
# build date/time/revision strings live in .rodata, compare the rest
strings -n 4 "$portElf" > "$out/port.str"
if grep -i -q -E 'indiflight|FORKNAME' "$out/port.str"; then
    echo "branding: indiflight strings found in stock-mode build"; grep -i -E 'indiflight|FORKNAME' "$out/port.str"; status=1
else
    echo "branding: none"
fi
[ $status -eq 0 ] && echo "identical"
exit $status
