# Test fixtures (recorded inputs)

Trimmed copies of real runs; `logs/` is gitignored, so CI needs these.
Golden OUTPUTS live in `tests/golden/` and were produced by the code at the
refactor branch point (commit 44c1f178) from these inputs; regenerate only
for an intentional behaviour change: `python3 tests/test_golden.py --regen`.

| dir | source run | contents |
|---|---|---|
| `mapping_run/` | `logs/run_20260813_142039` (Gazebo, 2 mappers) | all 25 waypoint sidecars (planner input) + the frames that had weed detections |
| `sar_gazebo_run/` | `logs/run_20261108_094434` (Gazebo SAR, drone 3 found the person) | sidecars, frames, `sar/call.json`, `sar/status_*.json` |
