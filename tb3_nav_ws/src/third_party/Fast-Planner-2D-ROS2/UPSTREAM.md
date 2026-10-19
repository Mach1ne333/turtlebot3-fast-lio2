# Upstream provenance

This source tree is the integration fork used by the OpenSpec change
`integrate-fast-planner-controller-with-nav2`.

- Upstream: https://github.com/JackJu-HIT/Fast-Planner-2D-ROS2.git
- Pinned commit: `9ed668e9d7f47292b7256a1ee539f0164bf664f0`
- Upstream default branch at import: `master`
- Imported for: ROS 2 Humble / Nav2 TurtleBot3 Burger integration
- Root license file SHA-256 at import: `8635cc22465235ef9483a8fae5e981c4b2a2b8a1a182b8475b46c32a27460445`

Local tracked and untracked changes relative to the pinned revision are integration patches. Preserve the
upstream root `LICENSE` and `fast_planner/LICENSE` files when redistributing.
This repository vendors the already-patched tree without its old `.git` directory.
`upstream-integration.patch` records tracked changes against that revision at migration;
`MIGRATION_SHA256.json` records the exact imported core and adapter files (before
repository-local README path updates). Newly added regression tests are included in
that manifest. Do not reapply the patch to this already-patched tree.
The original source workspace is retained unchanged and is not a runtime dependency.

## Algorithm provenance and integration contract

This is a third-party extraction/adaptation, not the official
[HKUST Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner).
Its README credits [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan).
The integration does not claim paper reproduction or import the official search front end.
Nav2 retains global planning; this library supplies local geometric optimization only.

The integration entry is `PlannerInterface::makeGeometricPlan(options, deadline)`.
Results carry positions; `PathPoint.v` is zero and must not be interpreted as an executable
speed profile. `makePlan()` remains a convenience entry for the visualization demo,
with the same geometric semantics. Actual Burger tracking belongs to `fast_planner_nav2`.

Patches are limited to core export/dependencies; explicit result, exception and deadline
handling; degenerate-input and ESDF boundary checks; consistent vertical obstacle extrusion;
per-call optimization weights/parameters; and removing the forced safety success.
The fixed-end objective's terminal acceleration penalty now uses the same positive-violation
condition as its gradient. Its parameter values no longer persist from the first instance.
The cost model is otherwise retained. Small library regression tests cover those changes.

Legacy free-end/jerk/demo-only optimization paths are not certified by this integration.
Nav2 does not use the legacy dynamic time-adjustment/replanner path.

## Test scope

The default test run retains all functional regression tests. Style checks in this
third-party package cover `test/`, top-level `CMakeLists.txt`, and `package.xml`.
The user approved excluding whole-tree legacy style cleanup from this integration;
this is not a declaration that old generated headers or unrelated code pass lint.
Use `-DFAST_PLANNER_LINT_UPSTREAM=ON` to opt back into the original whole-tree checks.
The Nav2 adapter and bringup packages retain their full checks.

Cppcheck registration is unchanged. Humble's ament wrapper skips installed cppcheck
2.7 by default due to known performance issues; such results are **skipped**, not
successful static analysis. No static-analysis coverage is claimed for skipped files.
