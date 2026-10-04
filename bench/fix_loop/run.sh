#!/usr/bin/env bash
# Regenerate a fix-loop run from raw inputs with the pipeline code exactly as tagged.
#   bash bench/fix_loop/run.sh before    # pipeline at tag fixloop-before
#   bash bench/fix_loop/run.sh after     # pipeline at tag fixloop-after
#   bash bench/fix_loop/run.sh 2-after   # pipeline at tag fixloop2-after (iteration 2)
# The benchmark/compare scripts are the current ones (identical metric for both runs).
set -euo pipefail
TAG="fixloop-$1"
[[ "$1" == 2-* ]] && TAG="fixloop${1}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WT="$ROOT/.worktrees/$TAG"
[ -d "$WT" ] || git -C "$ROOT" worktree add -f "$WT" "$TAG" >/dev/null
export ROOMSCAN_SRC="$WT/src"
python "$ROOT/bench/run_benchmark.py" --force --tiers lidar --no-damage \
  --runs "$ROOT/out/fix_loop/$1" --out "$ROOT/bench/fix_loop/$1"
PYTHONPATH="$ROOT/src" python "$ROOT/bench/fix_loop/evidence.py" "$ROOT/out/fix_loop/$1/lidar" \
  "$ROOT/bench/fix_loop/$1/overlay.png" > "$ROOT/bench/fix_loop/$1/evidence.txt"
echo "done: bench/fix_loop/$1/BENCHMARK.md"
