#!/usr/bin/env bash
# Full flight smoke test against a running SITL.
#
# WARNING: this arms the vehicle, takes off and flies a short forward leg.
# Only run it in a controlled SITL (ROS_DOMAIN_ID=42) with clear airspace.
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DCTL="${ROOT_DIR}/bin/dctl"

cleanup() {
  echo "!! cleanup: stop + land"
  "${DCTL}" stop >/dev/null 2>&1 || true
  "${DCTL}" land --timeout 60 >/dev/null 2>&1 || true
}
trap cleanup ERR INT TERM

echo "== 1. status =="
"${DCTL}" status --human

echo "== 2. arm =="
"${DCTL}" arm --human

echo "== 3. takeoff 2 m =="
"${DCTL}" takeoff --alt 2 --human

echo "== 4. offboard start =="
"${DCTL}" offboard start --human

echo "== 5. forward 1 m/s for 2 s =="
"${DCTL}" velocity --forward 1 --duration 2 --human

echo "== 6. hover 1 s =="
"${DCTL}" hover --duration 1 --human

echo "== 7. stop =="
"${DCTL}" stop --human

echo "== 8. land =="
"${DCTL}" land --human

echo "== 9. disarm =="
"${DCTL}" disarm --human

trap - ERR INT TERM
echo "FLIGHT SMOKE OK"
