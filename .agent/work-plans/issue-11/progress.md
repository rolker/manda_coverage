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

## Implementation
**Status**: complete
**When**: 2026-11-24 09:28 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-11 at `aba24ff`

Seven code and CI commits in the plan's order, with the checkpoint's five corrections applied; none touches `.agent/`, each ends "Backport to jazzy.":

1. `40bcf63` CMake: `target_link_libraries(... nav2_util::nav2_util_core Eigen3::Eigen)` and the test's `nav2_util::nav2_util_core` (#415 cause 1).
2. `7d13db9` `tf2/utils.hpp` (cause 2).
3. `a62600c` drop the unused `nav2_util/node_utils.hpp` include.
4. `36ad67b` `__has_include` switch and alias `manda_coverage::Nav2LifecycleNode` in `action_server.h`; base class and initialiser use it; `main.cpp` comment says "Nav2's". `nav2_ros_common` named in neither CMake nor `package.xml` (causes 7/13).
5. `ae86bda` the guard at `src/RecordSwath.cpp:60` (`else if (!m_min_record.empty())`) and `RecordSwathFirstInterval.KeepsFirstRecordOfLine` (owner's decision: "Take the guard, both branches").
6. `bebd711` `package.xml`: `<depend>` geometry_msgs, libboost-dev, rcl_interfaces, tf2, tf2_geometry_msgs; `<exec_depend>` launch, launch_ros, lifecycle_msgs (cause 11). `rosdep resolve` OK for jazzy/noble and rolling/resolute; `ament_xmllint`: No problems found.
7. `f84bea2` `.github/workflows/ci.yml`: one Jazzy job; its header says why there is no Rolling job.

Records: `da44dad` `hand_run.sh`; `aba24ff` `coverage_probe.py` and `hand_run_rolling/` (the recorded run); this entry.

### Verification on Rolling (package worktree; GCC 15.2, empty build type = -O0)

- `./build.sh --parallel-workers 4 --packages-up-to manda_coverage` (clean): `Summary: 1 package finished [18.5s]`; 44 compiler warnings, the same 44 as after steps 1-4 alone (also a clean build) and as the plan's baseline; none new, no deprecations.
- `./test.sh --parallel-workers 4 --packages-select manda_coverage --ctest-args -LE linter`: `100% tests passed, 0 tests failed out of 2`; `Summary: 13 tests, 0 errors, 0 failures, 0 skipped` (test_parameters 10/10 with the new test, test_control_server_lifecycle 1/1; the 13 count includes ctest's own records).
- Each gtest binary run 20 times as separate processes (ROS_DOMAIN_ID 187): test_parameters 20/20, test_control_server_lifecycle 20/20.
- Linters (not run in CI, recorded for the follow-up): copyright 36 failures of 38 (the 34 source files of the baseline plus LICENSE and CONTRIBUTING.md, which Rolling's ament_copyright also checks), uncrustify 34 of 35, cpplint 1,838 of 1,840, flake8 5 of 5, cppcheck skips all 35; lint_cmake, pep257, xmllint pass. No new failure: the new test lines are uncrustify- and cpplint-clean (`test_parameters.cpp`'s uncrustify diff is only in pre-existing lines 170-187).

### The guard (commit 5): what it changes

The unguarded code read `m_min_record.back()` on an empty list (undefined behaviour, in the code since 2016) for every record from the second up to the first `MinInterval()` of each survey line. GCC 15's unoptimised build aborts there, so on Rolling's default build every coverage goal killed the node; on other builds the read returned a heading near 0 and, when the heading and output side met the turn condition (heading above 20 with Port, below -20 with Stbd), the line's first record was dropped. The guard keeps the first record, as `MinInterval()`'s own comment intends, and so changes where the next line runs near the start of the recorded line in the geometries where the old override fired.

Regression test without the guard (scratch copy, guard removed, the test's own library forced with `LD_LIBRARY_PATH`):
- `-O0` (GCC 15): `RecordSwathFirstInterval.KeepsFirstRecordOfLine` aborts: `stl_list.h:1674: ... back() ... Assertion '!this->empty()' failed.`, exit 134.
- `-O2`: the test fails its assertions at `test/test_parameters.cpp:274` and `:275` (first outer point x 86.564971 vs expected 85.857864, y 214.849242 vs 214.142136: the second record's outer point, 1 m on); 9 passed, 1 failed.
- `-O0 -D_GLIBCXX_NO_ASSERTIONS` (unoptimised without libstdc++'s assertions, the nearest to Jazzy's GCC 13 here): the same failure at lines 274 and 275.
- With the guard: passes at -O0 (and in all 20 repeats).

Size of the change (scratch harness replaying `SurveyPath`'s flow on `RecordSwath` + `PathPlan`, compiled at -O2, guarded against unguarded; 200 x 100 m rectangle rotated 30 degrees, float32 vertices, 1 m records, lead-in 15 / lead-out 5, overlap 0.2, bend 60, interval 10; "fwd" = counter-clockwise, first line Port at heading 60, where the old override fires on every line; "rev" = reversed list, clockwise, where it never fires):

| order | swath per side | result |
|---|---|---|
| fwd | constant 20 m | identical lines (3 lines) |
| fwd | 20 + 4 sin(2 pi a/60) (16-24 m) | line 2: same vertex count (18), up to 0.45 m apart over its last 25 m (the end nearest the start of the recorded line); line 3: up to 0.96 m, along its whole length (planned from the changed line 2) |
| fwd | 20 - 4 sin(2 pi a/60) | line 2 identical; line 3 up to 0.11 m over its last 37 m; line 4 up to 0.26 m |
| fwd | 24 falling to 16 over 240 m | same geometry (0.000 m), different vertex counts: line 2 11 vs 8, line 3 9 vs 22 (collinear points) |
| rev | all four profiles | identical |

These are smaller than the plan review's measurement (up to about 5 m over the last 40 m, and two extra collinear vertices with a constant swath); the size depends on the swath profile over the first interval and on the run's details. Both runs agree on the shape: one vertex order only, and only near the start of the recorded line, compounding into later lines.

### Hand run on Rolling

`ROS_DOMAIN_ID=187`, checked empty with `ros2 node list --no-daemon`. Command:
`.agent/work-plans/issue-11/hand_run.sh /home/roland/project11/worktrees/issue-project11-rolker-manda_coverage-11/env.sh .agent/work-plans/issue-11/hand_run_rolling 187`
which runs `manda_coverage_action_server`, then the underlay's source-built `lifecycle_manager --ros-args -r __node:=lifecycle_manager_coverage -p "node_names:=['manda_coverage_action_server']" -p autostart:=true`, then `coverage_probe.py fwd <out> 300` and `coverage_probe.py rev <out> 300` on the same node. Exit 0.

- Lifecycle: `active [3]` after bring-up and after both goals (bond held throughout). `ros2 param list` shows the seven coverage parameters plus Rolling's `warn_on_missing_params` and `strict_param_loading`.
- fwd: `success=True`, 3 lines; offsets 0, 36, 72 m, spacings 36.00, 36.00; coverage 92.0%; no NaN orientations; PASS.
- rev: `success=True`, 3 lines (numbered 4-6: the node's line counter is not reset between goals); offsets 100, 64, 28 m, spacings 36.00, 36.00; coverage 92.0%; no NaN orientations; PASS.
- The uncovered 8% is the far strip the planner leaves when the next offset line falls outside (the algorithm, as the plan says).
- Verdict: PASS on the plan's criteria, which are a sanity check for a flat swath, not a check of the planner's adaptive behaviour.
- Extra comparisons (scratch, same probe): an unguarded node built at -O2 gave lines identical to 1e-6 m in both orders (flat swath: the override fires in fwd but moves nothing, as in the harness); an unguarded node at the default -O0 aborted on the first line of the first goal (`Assertion '!this->empty()' failed`, `Aborted (core dumped)`), and the lifecycle manager reported "CRITICAL FAILURE: SERVER manda_coverage_action_server IS DOWN".
- No node left running; the domain was empty afterwards.

### CI rehearsal (Jazzy job, on this Rolling machine)

Rehearsed: the workspace symlink; the two `git clone --depth 1 -b jazzy` lines; `colcon list --paths-only --packages-up-to manda_coverage` (manda_coverage, marine_control, marine_control_interfaces, marine_nav_interfaces, marine_nav_utilities); the strict `rosdep install --ignore-src -y` as `--simulate --rosdistro jazzy --os ubuntu:noble` in a clean environment (rc 0, every key resolves, including ros-jazzy-nav2-util and ros-jazzy-nav-2d-utils); `ament_xmllint package.xml` (No problems found); build, test (`-LE linter`), the result-file check and `colcon test-result` with the same commands on Rolling, against the siblings' `rolling` branches and the underlay's Nav2 (5 packages built; 2/2 tests; both `.gtest.xml` found; `Summary: 13 tests, 0 errors, 0 failures, 0 skipped`); the YAML parses. Not rehearsable: the `ros:jazzy-ros-core` container, `apt-get`, `rosdep init/update` there, the Jazzy binaries, GCC 13, the siblings' `jazzy` branches building, and the `#else` branch of the lifecycle switch.

### Unverified on Jazzy

Nothing here ran on Jazzy (Ubuntu 26.04, GCC 15, no Docker). The pull request's first CI run is the first Jazzy evidence, this repository's first CI run, and the first run of the new regression test under GCC 13. The owner's `hand_run.sh` on deadpool (before and after the backport) is the only Jazzy hand run.

### Validation statement (for the pull request)

Only commit 5 changes behaviour: the guard removes undefined behaviour reached on every survey line and keeps each line's first record; in polygons where the boat's heading and the output side met the old turn condition (here one of the two vertex orders) it moves the next line near the start of the recorded line, measured at 0 m with a flat swath and up to about 0.5 m (1 m in the line after) with a varying swath in a scratch harness, and up to about 5 m in the plan review's harness; on Rolling's default build it is the difference between the node aborting on the first line and completing the survey. The other commits are build fixes and declarations. The lifecycle base class differs between the distros only in what the plan review listed (bond heartbeat period 0.25 s on Rolling vs 0.1 s on Jazzy, each consistent with its own lifecycle manager; the new `warn_on_missing_params` and `strict_param_loading`, unused here; an `enable_lifecycle_services` override; member `create_*` returning Nav2 wrappers, which this package does not call). The hand run shows the node brought up by Rolling's lifecycle manager with a bond, serving two goals, and planning parallel lines 36 m apart for a fake boat with a flat swath in both vertex orders. It does not show the behaviour tree or `mission_manager` calling the action, Nav2 following the lines, real sonar or TF, or a real survey.

### Left alone (for a follow-up issue; the owner's rule that a port makes no big changes)

- Lint: copyright 36 (34 files plus LICENSE, CONTRIBUTING.md), uncrustify 34, cpplint 1,838, flake8 5 (`launch/manda_coverage_launch.py`); cppcheck skips everything on Rolling. Excluded in CI.
- Compiler warnings (44 lines, 33 distinct): -Wunused-parameter `src/action_server.cpp:12,32,76,92,104,120,121,128`, `src/lib_geometry/XYPolygon.cpp:122`, `src/RecordSwath.cpp:84`; -Wreorder `PathPlan.h:244-252`, `RecordSwath.h:184-195`, `src/PathPlan.cpp:19`, `src/RecordSwath.cpp:23`; -Wsign-compare `src/SurveyPath.cpp:285,406,499,519`; -Wunused-variable `src/PathPlan.cpp:192,581,583`; -Wunused-label `src/PathPlan.cpp:319`; -Wcomment `src/PathPlan.cpp:486`; -Wcatch-value `src/PathPlan.cpp:809`.
- `nav_2d_utils` declared but unused: `CMakeLists.txt:15`, `package.xml:23`.
- `ament_export_dependencies` (`CMakeLists.txt:80`) lists only Eigen3, sensor_msgs, visualization_msgs: missing nav2_util, marine_nav_utilities, rclcpp, nav_msgs.
- `eigen` only a `build_depend` (`package.xml:15`) though the installed `PathPlan.h` includes Eigen.
- The installed `include/manda_coverage/action_server.h` (`CMakeLists.txt:66-69` installs `include/`) declares a class no library provides (`action_server.cpp` is in the executable and the test only).
- `handle_cancel` (`src/action_server.cpp:127`) never calls `canceled()` on the goal handle.
- `respawn=True` (`launch/manda_coverage_launch.py:18`) hides crashes.
- Rolling's "LifecycleNode is not shut down: Node still in state(1) in destructor" warning on SIGINT (in `hand_run_rolling/server.log`).
- The build type is empty (unoptimised) by default; nothing in the project sets it.
- The line counter (`line_number` in feedback) carries on across goals on the same node (seen in the hand run: 4-6 for the second goal).
- For rolker/unh_marine_navigation: `marine_nav_utilities::adjustPathOrientations` (`marine_nav_utilities/src/utilities.cpp:42`) gives NaN quaternions for a segment along the x axis. This package calls it at `src/SurveyPath.cpp:509` (510 before this branch removed an include), so the lines it returns can carry NaN orientations (none in this hand run: its lines are at 30 degrees). The other caller is `marine_nav_behavior_tree`'s `FixPathOrientations` (`src/plugins/action/fix_path_orientations.cpp:63`, `only_if_invalid`), which treats them as invalid and recomputes the same NaN; the consumer is `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp:923`, `tf2::getYaw` of the plan pose's orientation. The fix belongs in marine_nav_utilities.

### Differences from the plan and the checkpoint

- The plan review's numbers for the size of the change were not reproduced: this harness gives 0 m with a constant swath (no extra vertices) and under 1 m with three varying profiles; the shape (one vertex order, near the start of the recorded line) matches. The review's harness and profile are not in the records, so the difference is not explained.
- `hand_run.sh` names the order the guard can change ("fwd", counter-clockwise; "rev" is the reversed order and never fires here). With the probe's flat swath the Rolling comparison found no difference in either order, so the script says a difference is possible in fwd but may be nil, and a difference in rev is a bug.
- `hand_run.sh` sends both goals to one node (realistic, and it shows a second goal working) rather than restarting it.
- `coverage_probe.py` gives up when the action server has been gone for 10 s (added after the unguarded -O0 comparison left it waiting out its timeout).
- `SurveyPath.cpp:510` in the checkpoint is now line 509.
- The copyright count is 36, not 34: Rolling's ament_copyright also reports LICENSE and CONTRIBUTING.md.
