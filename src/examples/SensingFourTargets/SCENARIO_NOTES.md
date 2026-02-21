# SensingFourTargets Scenario Notes

This scenario contains 4 targets and is derived from `SensingTwoTargets`.

## Motion Model

- All target trajectories are Boulic-model-based.
- `TargetBase0.dat` / `TargetJoints0.dat` and `TargetBase1.dat` / `TargetJoints1.dat` are copied from `SensingTwoTargets`.
- `TargetBase2.dat` / `TargetJoints2.dat` are generated from target 0 by a rigid transform (random planar heading + random XY translation).
- `TargetBase3.dat` / `TargetJoints3.dat` are generated from target 1 by a rigid transform (random planar heading + random XY translation).

## Randomness Policy

- Generation for targets 2 and 3 is intentionally non-deterministic (fresh randomness each creation).

## Output Policy

- This folder intentionally commits only `Input/` files.
- No `Output/` folder is committed; generate outputs by running the simulator.

## Selection

Use in `src/main.m` by setting:

```matlab
scenarioNameStr = 'examples/SensingFourTargets';
```
