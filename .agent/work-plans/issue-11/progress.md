---
issue: 11
---

# Issue #11 — Port manda_coverage to ROS 2 Rolling

## Issue Review
**Status**: complete
**When**: 2026-11-24 13:45 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #11

### Scope Assessment

**Well-scoped?** Yes, with two gaps. One package, 2,359 lines outside the vendored `lib_geometry`/`lib_mbutil`; one PR plus a backport PR. The "known to need changing" table is incomplete: a first build on Rolling needs two more edits (below), and "Done when" says the tests pass without saying what that means for the lint tests (see Recommendations).
**Right repo?** Yes. The package is `rolker/manda_coverage`; the CI workflow belongs here too.
**Serves**: none (the project has no goals or design document); in the owner's terms it is the Rolling port, rolker/unh_marine_autonomy#415.
**Design picture?** No change. The package keeps its action (`compute_sonar_coverage_path`, `marine_nav_interfaces/action/ComputeSonarCoveragePath`) and its Nav2 lifecycle-node shape. No design document exists to update.
**Dependencies**: `marine_control`, `marine_control_interfaces`, `marine_nav_interfaces`, `marine_nav_utilities` (all built in the hosted core install); Nav2 `nav2_util`, `nav2_ros_common`, `nav_2d_utils` from the development underlay. Blocks rolker/ben_project11#30. Also an `exec_depend` of `unh_marine_simulation`'s `marine_simulation`.

**What the package is.** A sonar-coverage path planner ported from MOOS-IvP (`PathPlan`, `RecordSwath`, `SurveyPath`, plus vendored MOOS `lib_geometry` and `lib_mbutil`). `MandaCoverageActionServer` is a Nav2 lifecycle node serving `compute_sonar_coverage_path` and publishing paths; it adopts `marine_control`'s `ControlServer` for seven tuning parameters.

**On the mission path?** Yes. The BT plugin `sonar_coverage_action` in `unh_marine_navigation/marine_nav_behavior_tree` is the action client (used by `marine_nav_bt_task_navigator`'s `run_tasks.xml`). It is launched by `ben_project11` (`navigation_launch.py`, `nav2_params.yaml`; `exec_depend`) and by `unh_marine_simulation` (`vrx_project11/launch/navigation_launch.py`, `marine_simulation` demo launch files). The `mission_manager` `multibeam_coverage_adapter.py` in `unh_marine_autonomy` also names `ComputeSonarCoveragePath`; I did not read it closely, so whether it is a client of this package's action or only of the same message type is for the planner to check.

**Tests.** `test_parameters` (9 gtests, pass) and `test_control_server_lifecycle` (1 gtest, passes) with a hand-patched probe build; plus the `ament_lint_auto` set (copyright, cpplint, uncrustify, flake8, cppcheck, lint_cmake, pep257, xmllint). No test calls the planner or drives the action end to end.

**What a first Rolling build shows** (build run in the worktree; the later layers found by a throwaway patched copy in the scratchpad, since removed; nothing in the worktree was changed):

1. `CMakeLists.txt:60` `ament_target_dependencies` (cause 1); the same call at `:142` hides behind it.
2. `src/SurveyPath.cpp:23` `tf2/utils.h` missing (cause 2).
3. NOT IN THE ISSUE: `src/SurveyPath.cpp:24` `nav2_util/node_utils.hpp` missing (it moved to `nav2_ros_common/node_utils.hpp`; nothing from it is used in the file, so the include can go). Same family as cause 13.
4. `include/manda_coverage/action_server.h:6` `nav2_util/lifecycle_node.hpp`, and the base class at `:14` and `src/action_server.cpp:7` `nav2_util::LifecycleNode` (causes 7/13: `nav2_ros_common/lifecycle_node.hpp`, `nav2::LifecycleNode`; the test files include the same names). The probe needed `nav2_ros_common` in `find_package` and in the link lines.
5. Not a failure: the `nav2_util` and `nav_2d_utils` `find_package` calls still resolve.

With those edits applied in the probe, the package built and both gtests passed. Cause 7 (`create_subscription`) does not apply; `SurveyPath.cpp:77,85` already call the free `rclcpp::create_subscription` on node interfaces.

**`nav_2d_utils`** still exists on the underlay's Nav2 `main` at `nav2_dwb_controller/nav_2d_utils`. The package declares it (`package.xml`, `find_package`) but no source or header uses it; the `nav2_util` dependency is likewise only for the lifecycle node. The planner can decide whether to drop the unused dependencies or leave them (a port makes no big changes: leaving them is the default).

**Nav2 APIs used**: `nav2_util::LifecycleNode` (moves to `nav2::LifecycleNode`), `nav2_util/node_utils.hpp` (unused include). Nothing else.

**Lint tests.** On Rolling, with the probe build, the lint tests fail: copyright 34, uncrustify 34, cpplint ~1,838, flake8 5 (the launch file); cppcheck is skipped on Rolling (cause noted in #415). These are old style defects, nothing the port introduces. `ament_lint_common` is a `test_depend`, so "tests pass" cannot mean "all pass" without a decision.

**Jazzy CI is straightforward.** One package, `rosdep install` without `-r`. It must clone `rolker/marine_control` (for `marine_control` and `marine_control_interfaces`) and `rolker/unh_marine_navigation` (for `marine_nav_interfaces` and `marine_nav_utilities`, whose own dependencies pull in more of that repository's packages: `colcon list --packages-up-to manda_coverage` over the clone will show how much). `marine_nav_utilities` may need `geodesy` from `geographic_info` at `f70b81a` (cause 8); to be checked in the plan. Nav2 comes from released Jazzy binaries (`ros-jazzy-nav2-util`, `ros-jazzy-nav-2d-utils`), per the owner's decision. No Rolling job.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | A port that makes no big changes; follow-ups listed rather than done. |
| Keep one current design | OK | No design document in the repository; no design change. |
| Put each thing at the level it applies to | OK | Fixes are in this package; the backport is by hand, per the owner. |
| Only what's needed (the issue's list is complete enough to plan from) | Watch | The first Rolling build finds two things the issue does not list (the `node_utils.hpp` include and the lint test failures); the issue's "known to need changing" table should not be taken as complete. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0013 (progress timeline) | Yes | Records go in `.agent/work-plans/issue-11/progress.md`, carried in the PR; not in a code commit. |
| Others | No | No workspace infrastructure changes. |

### Consequences

- `ben_project11` issue 30 waits on this one.
- A Jazzy CI workflow is added to both branches, in a separate commit from the code.
- On Rolling the package is checked by hand against the source-built Nav2 in the development underlay, with the runs recorded (build, `colcon test`, and ideally one lifecycle bring-up of the action server).

### Recommendations

- Add to the issue's cause table: `src/SurveyPath.cpp:24` `nav2_util/node_utils.hpp` (moved; the include is unused), and the lifecycle-node includes in `include/manda_coverage/action_server.h:6` and in both test files, not only `src/action_server.cpp:7`.
- Settle in the plan what "its tests pass" means: the two gtests must pass; the `ament_lint_auto` tests fail by the hundreds on old style defects (and cppcheck is skipped on Rolling). The default that fits "no big changes" is to run the gtests as the pass criterion in CI and record the lint result for the follow-up issue; this is the planner's call unless the owner wants the lint cleaned.
- Cause 7 and the Nav2 `create_subscription` candidate named in #415 do not apply here (the code already calls the free `rclcpp::create_subscription`); say so in the follow-up record so #415's table can be corrected.
- Use the `__has_include("nav2_ros_common/lifecycle_node.hpp")` switch of cause 13 (header and type alias) so the code builds on Jazzy, and gate the `nav2_ros_common` CMake and `package.xml` dependency the same way (Jazzy has no `nav2_ros_common` package).
- Plan a hand check on Rolling beyond the unit tests: bring the action server up through its lifecycle transitions against the source-built Nav2 and send one `ComputeSonarCoveragePath` goal; no existing test exercises the planner or the action path, and the owner's #415 notes say no simulation has been run.
- Check whether `unh_marine_autonomy`'s `multibeam_coverage_adapter.py` is a client of this action, and record the answer in the plan.

### Actions
- [ ] Only what's needed (the issue's list is complete enough to plan from): the first Rolling build finds two things the issue does not list (the `node_utils.hpp` include and the lint test failures); the "known to need changing" table is not complete.
- [ ] Add to the issue's cause table: `src/SurveyPath.cpp:24` `nav2_util/node_utils.hpp` (moved; the include is unused), and the lifecycle-node includes in `include/manda_coverage/action_server.h:6` and in both test files, not only `src/action_server.cpp:7`.
- [ ] Settle in the plan what "its tests pass" means: the two gtests must pass; the `ament_lint_auto` tests fail by the hundreds on old style defects (and cppcheck is skipped on Rolling). The default that fits "no big changes" is to run the gtests as the pass criterion in CI and record the lint result for the follow-up issue; this is the planner's call unless the owner wants the lint cleaned.
- [ ] Cause 7 and the Nav2 `create_subscription` candidate named in #415 do not apply here (the code already calls the free `rclcpp::create_subscription`); say so in the follow-up record so #415's table can be corrected.
- [ ] Use the `__has_include("nav2_ros_common/lifecycle_node.hpp")` switch of cause 13 (header and type alias) so the code builds on Jazzy, and gate the `nav2_ros_common` CMake and `package.xml` dependency the same way (Jazzy has no `nav2_ros_common` package).
- [ ] Plan a hand check on Rolling beyond the unit tests: bring the action server up through its lifecycle transitions against the source-built Nav2 and send one `ComputeSonarCoveragePath` goal; no existing test exercises the planner or the action path, and the owner's #415 notes say no simulation has been run.
- [ ] Check whether `unh_marine_autonomy`'s `multibeam_coverage_adapter.py` is a client of this action, and record the answer in the plan.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 08:41 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the issue review found the issue well scoped and clear enough to plan from; its findings are corrections and choices for the planner, and none changes what the work is for or its scope)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review, status complete. For the planner: the issue's table is incomplete: `src/SurveyPath.cpp:24` includes `nav2_util/node_utils.hpp`, which is unused and no longer exists on Rolling, and the lifecycle-node include is also in `include/manda_coverage/action_server.h:6` and both test files; the `__has_include` switch of #415 cause 13 applies, with `nav2_ros_common` found and linked only where it exists. Cause 7 as first described does not apply here. Pass criterion: the two gtests on both distros; the `ament_lint_auto` tests fail in bulk on the tree as it stands (copyright, uncrustify, cpplint, flake8) and are not this port's: the plan says how CI treats them (excluded, as rolker/unh_marine_navigation's workflow does) and lists them for the follow-up issue. Dependencies that are declared but unused (`nav_2d_utils`) are left as they are. The package is on the mission path (the behaviour tree's sonar coverage action calls it) and no test exercises its planner or action: the plan includes one hand run on Rolling against the source-built Nav2 that brings the lifecycle node up and sends one `ComputeSonarCoveragePath` goal, recorded, and says what that does and does not show. The Jazzy CI job clones what the package needs (rolker/marine_control, rolker/unh_marine_navigation, `geodesy` if the closure needs it) and uses released Nav2 binaries; there is no Rolling job (owner, 2026-11-24: no CI job builds Nav2 source).

## Plan Authored
**Status**: complete
**When**: 2026-11-24 08:58 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-11/plan.md` at `76fb1c7`

Seven commits into `rolling`, each backported: CMake targets for cause 1, `tf2/utils.hpp`, drop the unused `nav2_util/node_utils.hpp`, a `__has_include` switch and alias in `action_server.h` (no `nav2_ros_common` in CMake or the manifest: it comes through `nav2_util` on Rolling, as in `ais_layer`/`bathymetry_layer`), a guard for `RecordSwath::AddRecord`'s read of an empty list (found by a closed-loop trial: GCC 15 at `-O0` aborts the node on every survey; owner to confirm), undeclared dependencies, and a Jazzy-only CI workflow. Scratch build with steps 1-4 passes both gtests on Rolling; a closed-loop hand run with a fake boat and Rolling's lifecycle manager is planned, with a script for the owner's Jazzy comparison.

## Plan Review
**Status**: complete
**When**: 2026-11-24 09:01 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: needs-work

**Issue**: #11 — Port manda_coverage to ROS 2 Rolling
**Plan**: `.agent/work-plans/issue-11/plan.md` at `76fb1c7`
**Branch**: `feature/issue-11`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One package, seven small commits, one per cause; records kept apart. |
| Issue alignment | Needs work | Every issue-review action is addressed (node_utils include, lint excluded and listed, `__has_include` switch, hand run with a goal, `multibeam_coverage_adapter.py` confirmed a client). Serves line, as checked: "None (the project has no goals or design document); the Rolling port, rolker/unh_marine_autonomy#415." The step 5 question put to the owner rests on a claim that is true only for the trial's geometry (finding 1). |
| File targeting | Good | Files match the causes; test files do not name Nav2 (checked: only `src/main.cpp:27` comment and `action_server.h`/`.cpp`). |
| Consequences | Needs work | `core.repos` pin handed to the host; backport by hand. The Jazzy comparison's expectation is too broad (finding 4); the left-alone record misses that this package itself calls `adjustPathOrientations` (finding 5). |
| Principle alignment | Needs work | "Test what breaks": the step 5 test fails only on GCC 15 at `-O0`, never in the Jazzy CI this plan adds (finding 2). "Name the rule before bending it": done for step 5. |
| ADR compliance | Good | ADR-0013 only; records in their own commits. |
| ROS conventions | Good | Lifecycle base switch as in `ais_layer`/`bathymetry_layer` (theirs is in tests only; here it is in a public header, which is where the class is declared). Base-class differences checked: Rolling's node and manager both default `bond_heartbeat_period` 0.25, Jazzy's both 0.1, so each distro is consistent with itself. |

### Verified

- Cause 1: Rolling `nav2_util_coreExport.cmake` links `nav2_ros_common::nav2_ros_common` in `INTERFACE_LINK_LIBRARIES`, and `nav2_util`'s exported dependencies include `nav2_ros_common`, so naming only `nav2_util` works on Rolling. Jazzy `c7046550`: `library_name ${PROJECT_NAME}_core`, `ament_export_targets(${library_name})`, one `add_library`, so `nav2_util::nav2_util_core` is the name there too.
- CI closure over the `jazzy` branches: manda_coverage, marine_control, marine_control_interfaces, marine_nav_interfaces, marine_nav_utilities; no `geodesy`, none of the six Nav2 packages. All eight `ament_cmake_*` linters set `LABELS "<name>;linter"`, so `-LE linter` drops all eight.
- No repository in `rolling/core/src` sets `CMAKE_BUILD_TYPE` in a workflow or script; `rolling/core/build/*/CMakeCache.txt` all have it empty; CI builds will be `-O0`.
- GCC 15.2 (`/usr/include/x86_64-linux-gnu/c++/15/bits/c++config.h:601`): `_GLIBCXX_ASSERTIONS` is defined when `__OPTIMIZE__` is not. At `-O2` nothing defines it on this machine. GCC 13's `c++config.h` has no such block (added in GCC 15), so Jazzy at `-O0` does not assert.
- `RecordSwath::AddRecord`: after `ResetLine()` (start of every line, `SurveyPath.cpp:289,482`) `m_has_records` is false, so the first record skips the block; the second record, if within `swath_record_interval` (default 10 m) of the first, reads `m_min_record.back()` on an empty list. Reached on every line. `MinInterval()` always pushes once `m_interval_record` is non-empty (it always holds the current record), and only `ResetLine()` empties `m_min_record`, so after the first interval the read is always safe: the guard can act only before the first `MinInterval()` of each line.
- Without assertions, `back()` on the empty list reads the list's own header node as a node: `loc_x` is the size (0), `loc_y`/`heading` are the following member, `m_interval_swath`'s red-black header (`_M_color` 0 in the low half, padding in the high half). Probe at `-O2`: heading 0.0 on zeroed memory, -2.5e-98 on 0xAB-filled memory. So the unguarded build compares against a heading of about 0, deterministically in practice: the "turn" is the boat's compass heading, and the override fires on the second record of each line when the heading is above 20 with output side Port, or below -20 with side Stbd.
- The far-strip claim: `PathPlan::GenerateNextPath` offsets from the swath edge by `swath*(1-margin)` and `ClipToRegion` drops points outside; at one point or fewer the survey ends, so a strip narrower than the next offset is left. That is the algorithm.
- `adjustPathOrientations` (`marine_nav_utilities/src/utilities.cpp:42`): a segment with dy == 0 gives `rotation_axis` zero; `tf2::Quaternion::setRotation` divides by its length (`tf2Assert` is empty without `TF2_DEBUG`), so x, y, z are NaN. Real.

### Findings

1. **[Issue alignment] Medium**: The plan says the guarded result "matches today's Jazzy output" (Context and Open Questions). It does not in general. In the unguarded build the spurious override fires on the second record of every line whenever the compass heading and output side meet the turn condition, pushing that record as the first minimum and dropping the line's first record; with the guard the first record is kept (what `MinInterval()`'s "Add the first point if this is the first interval" intends). Reproduced with `RecordSwath` and `PathPlan` built at `-O2` with and without the guard on a 200 x 100 m rectangle: identical next lines for one vertex order, different for the reversed order (30 degree rotation, constant swath: two extra collinear vertices at the line end; varying swath 16 to 24 m: the last 40 m of the next line moved by up to about 5 m). The planner's 1e-14 match is a property of its trial's polygon orientation. Resolution: restate step 5 and the open question: the guard removes undefined behaviour and keeps the line's first record, and in about half of geometries it changes where the next line runs near the start of the recorded line, in the direction the original code intends.
2. **[Principle alignment] Medium**: The step 5 test "adds two records closer than the interval to a fresh RecordSwath". It reaches the read (checked: fresh object, two records 1 m apart, aborts at `-O0` on GCC 15 with `Assertion '!this->empty()'`, survives with the guard), but it fails only on GCC 15 unoptimised; in the Jazzy CI this plan adds, and in any Release build, it passes without the guard. Resolution: add an observable assertion that fails without the guard on any build in practice, e.g. set output side Port, feed a straight line at heading 45 with records 1 m apart past one interval, and assert that the first `SwathOuterPts(Port)` point is the first record's outer point (unguarded at `-O2` it is the second record's). Say in the test that the unguarded value comes from undefined behaviour.
3. **[Issue alignment] Low**: "the first record of a line skips the turn override" is imprecise: every record from the second to the first `MinInterval()` of each line (the first `swath_record_interval`, 10 m by default) skips it. Say so in step 5 and in the commit message.
4. **[Consequences] Low**: The hand run's Jazzy comparison ("diff the line files; expected: identical within 1e-6", before and after the backport) holds only for a non-firing geometry, and a flat 40 m swath does not exercise the adaptive offset where the guard's effect shows. Resolution: run `coverage_probe.py` with the polygon in both vertex orders (or headings of both signs) and expect the reversed order to differ after the backport; or state that the comparison covers one geometry. The other judging criteria (parallel lines, 32 to 40 m spacing, vertices inside) are a fair sanity check for a flat swath, not a check of the planner's adaptive behaviour; say so in "Not shown".
5. **[Consequences] Low**: The left-alone entry puts the NaN quaternions in rolker/unh_marine_navigation, but this package calls `marine_nav_utilities::adjustPathOrientations(next_path.poses)` itself in `SurveyPath::sendPath` (`src/SurveyPath.cpp:510`), so the lines this node returns carry NaN orientations for a segment along +x or -x. Downstream, `FixPathOrientations` (`marine_nav_behavior_tree`, `only_if_invalid`) treats them as invalid and recomputes the same NaN, and `crabbing_path_follower.cpp:923` takes `tf2::getYaw` of the plan pose's orientation. Resolution: the follow-up draft names both callers and the consumer; the fix still belongs in `marine_nav_utilities`.

### Summary

The port steps (causes 1, 2, 4, 6, the unused include, the CI job) are right and checked against the underlay and Jazzy sources. Step 5 is a real fix of real undefined behaviour reached on every survey line, but the plan's case for it says the guard changes nothing relative to Jazzy, which holds for one polygon orientation only; the owner should decide on the corrected facts, and the test should fail without the guard somewhere other than GCC 15 at `-O0`.

### Recommended Actions

- [ ] Restate step 5 and its open question with finding 1's facts (finding 1).
- [ ] Add an observable assertion to the step 5 test that fails without the guard in a Release or GCC 13 build (finding 2).
- [ ] Say "records before the first interval of each line", not "the first record" (finding 3).
- [ ] Run the hand probe in both polygon orientations, or narrow the Jazzy comparison's expectation; note the flat swath in "Not shown" (finding 4).
- [ ] Name `SurveyPath.cpp:510` and the downstream consumers in the `adjustPathOrientations` follow-up draft (finding 5).

## Checkpoint
**Status**: complete
**When**: 2026-11-24 09:04 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: plan
**Decision**: proceed

The plan review's verdict is needs-work, on five findings that each have a concrete correction; the one question in it that was the owner's has his answer, so the implementer applies the corrections and no second plan round is run. **The owner's decision on step 5** (the guard at `src/RecordSwath.cpp:60`), 2026-11-24, shown that the read of an empty list is reachable on every survey line, that GCC 15's unoptimised build aborts on it, and that the guard changes the next line near the start of the recorded line in about half of geometries (measured: two extra collinear points with a constant swath; up to about 5 m over the last 40 m with a varying swath): "Take the guard, both branches (Recommended)". So the guard goes in, in its own commit, and is backported; the pull request states it as a behaviour change on the mission path. Corrections, overriding the plan: (1) nowhere say the guarded result "matches today's Jazzy output": it removes undefined behaviour and keeps the line's first record, as `MinInterval()`'s own comment intends, and it changes planned lines where the old code's spurious override fired. (2) The regression test must fail without the guard on any build, not only on GCC 15 unoptimised: output side Port, a straight line at heading 45 with records 1 m apart past one interval, assert that the first `SwathOuterPts(Port)` point is the first record's outer point; the test says that the unguarded value comes from undefined behaviour; prove it by removing the guard in both an unoptimised and an optimised build. (3) Say precisely which records skip the turn override: every record from the second up to the first `MinInterval()` of each line. (4) The hand run uses the polygon in both vertex orders and records both; the Jazzy-side script says that the reversed order is expected to differ after the backport, and its judging criteria are described as a sanity check for a flat swath, not a check of the planner's adaptive behaviour. (5) The left-alone list names both callers of `marine_nav_utilities::adjustPathOrientations` and its consumer: this package calls it at `src/SurveyPath.cpp:510`, so the lines this node returns carry NaN orientations for a segment along the x axis; the fix belongs in rolker/unh_marine_navigation and is not made here. Host decisions: all undeclared dependencies are declared in the cause 11 commit; lint stays excluded in CI.
