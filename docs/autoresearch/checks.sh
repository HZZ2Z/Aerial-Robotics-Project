#!/usr/bin/env bash
# FROZEN correctness gate (write-protected by the autoresearch hook): validates
# .auto/bench_last.txt written by .auto/bench_ertk.m in the same benchmark run.
set -e
cd "$(dirname "$0")"
[ -s bench_last.txt ] || { echo "checks FAIL: bench_last.txt missing"; exit 1; }
eval "$(sed 's/\r$//; s/^/B_/' bench_last.txt)"
chk() {
  awk -v v="$2" -v lim="$3" 'BEGIN{ if (v==v && v>0 && v<lim) exit 0; exit 1 }' \
    || { echo "checks FAIL: $1=$2 (need 0 < v < $3)"; exit 1; }
}
chk ertk_rms       "$B_ertk_rms"       1.0
chk ewind_rms      "$B_ewind_rms"      0.095
chk tilt_ertk_deg  "$B_tilt_ertk_deg"  35.0
chk tilt_ewind_deg "$B_tilt_ewind_deg" 35.0
echo "checks OK"
