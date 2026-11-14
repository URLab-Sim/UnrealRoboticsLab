#!/bin/bash
# Prefix BoringSSL's global symbols in the staged gRPC archives so a monolithic
# Unreal target can link them alongside the engine's OpenSSL. Idempotent.
set -euo pipefail

LIB_DIR=${1:?usage: prefix_boringssl.sh <grpc lib dir>}
PREFIX=${BORINGSSL_SYMBOL_PREFIX:-urlab_bssl_}
NM=${NM:-llvm-nm}
OBJCOPY=${OBJCOPY:-llvm-objcopy}

for TOOL in "$NM" "$OBJCOPY"; do
	command -v "$TOOL" >/dev/null 2>&1 || {
		echo "prefix_boringssl.sh: $TOOL not found; install LLVM binutils or set NM/OBJCOPY" >&2
		exit 1
	}
done

MAP=$(mktemp)
trap 'rm -f "$MAP" "$MAP".a' EXIT

# llvm-nm reports every symbol-less assembly object; keep real errors only.
"$NM" -g --defined-only --format=just-symbols "$LIB_DIR/libssl.a" "$LIB_DIR/libcrypto.a" \
	2> >(grep -v ": no symbols$" >&2) \
	| { grep -v -e ':$' -e '^$' -e "^$PREFIX" || true; } \
	| sort -u \
	| awk -v p="$PREFIX" '{ print $1 " " p $1 }' > "$MAP"

if [ ! -s "$MAP" ]; then
	echo "BoringSSL symbols in $LIB_DIR already prefixed"
	exit 0
fi

echo "Prefixing $(wc -l < "$MAP") BoringSSL symbols with $PREFIX"
for LIB in "$LIB_DIR"/*.a; do
	"$OBJCOPY" --redefine-syms="$MAP" "$LIB" "$MAP".a
	mv -f "$MAP".a "$LIB"
done
