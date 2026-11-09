#!/usr/bin/env bash
# Host-only unit tests for lib/turtle_recovery (no PX4 build needed).
set -euo pipefail
cd "$(dirname "$0")"
CXX="${CXX:-g++}"
FLAGS=(-std=c++14 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all -pthread)
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
for n in rc core fault_matrix mapping; do
	"$CXX" "${FLAGS[@]}" "test_$n.cpp" -o "$tmp/$n"
	"$tmp/$n"
done
"$CXX" "${FLAGS[@]}" test_interlock.cpp interlock_other.cpp -o "$tmp/interlock"
"$tmp/interlock"
echo "PASS: all turtle_recovery host suites"
