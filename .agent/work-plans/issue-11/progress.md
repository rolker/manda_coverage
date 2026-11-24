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
