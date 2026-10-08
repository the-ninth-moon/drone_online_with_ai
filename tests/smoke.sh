#!/usr/bin/env bash
# Read-only smoke test.  Safe to run any time: it never arms or takes off.
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DCTL="${ROOT_DIR}/bin/dctl"

echo "== status =="
"${DCTL}" status --human

echo "== watch (3 samples @ 2 Hz) =="
"${DCTL}" watch --rate 2 --count 3 --human

echo "== wait for landed =="
"${DCTL}" wait --for landed --timeout 5 --human

echo "SMOKE OK"
