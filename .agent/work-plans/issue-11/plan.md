# Plan: Port manda_coverage to ROS 2 Rolling

## Issue

https://github.com/rolker/manda_coverage/issues/11 (part of rolker/unh_marine_autonomy#415)

## Context

One C++ package, a Nav2 lifecycle node serving `compute_sonar_coverage_path`. It is on the
mission path: `marine_nav_behavior_tree`'s `SonarCoverageAction` (rolker/unh_marine_navigation)
calls it, and so does a second client, `mission_manager`'s `multibeam_coverage_adapter.py`
(rolker/unh_marine_autonomy: `ActionClient(..., ComputeSonarCoveragePath,
'compute_sonar_coverage_path')`). Neither client's code is touched or needed here: the action
type is in `marine_nav_interfaces`, not in the six Nav2 packages that hold the field server's
unimported work.

**Baseline** (a scratch copy of `rolling` at `d2a2d64`, built against `/opt/ros/rolling`, the
underlay and the core install): configure fails at `CMakeLists.txt:60`, `Unknown CMake command
"ament_target_dependencies"` (cause 1). The issue review's later layers were re-found with the
planned changes: `tf2/utils.h` (cause 2), `nav2_util/node_utils.hpp` (unused, gone),
`nav2_util/lifecycle_node.hpp` and `nav2_util::LifecycleNode` (causes 7/13). Correction to the
review: the test files do not name Nav2; they include `action_server.h`, so the switch lives in
that header only. With steps 1 to 4 below on the scratch copy: builds (44 old warnings, none
new, none deprecations), `test_parameters` 9/9 and `test_control_server_lifecycle` 1/1 pass;
`xmllint`, `lint_cmake` and `pep257` pass; copyright 34, uncrustify 34, cpplint 1,838 and
flake8 5 failures; cppcheck skips all 35 files.

Nav2 used: the underlay's navigation2 install, built 2026-11-19 from `dcffa939` (`main`); the
source checkout is now at `9113449e`, and `git diff dcffa939 9113449e -- nav2_ros_common
nav2_util` is empty, so the symlinked headers match the built libraries.

Other #415 causes checked: 14 (no `TransformListener`/`TransformBroadcaster` anywhere), 16 (no
`rclcpp::Waitable`), 7 (all subscriptions, publishers, timers and the action server use free
functions on node interfaces, so Nav2's hiding members are never reached), 9 (`xmllint` passes;
format 2 manifest, well-formedness only), 10 (GCC 15 compiles everything), 11 (below, step 6),
4/5/15 (no Python package; the launch file has no logger).

**A new finding, not a build failure.** A closed-loop trial in the scratchpad (goal, then a fake
boat publishing `odom` and `soundings` along each fed-back line) aborted the node on its first
survey line: `stl_list.h:1674 ... back() ... Assertion '!this->empty()' failed`.
`RecordSwath::AddRecord` (`src/RecordSwath.cpp:60`) reads `m_min_record.back()` before any
minimum is recorded. That is undefined behaviour on every distro; GCC 15's libstdc++ asserts on
it in unoptimised builds, and the project builds with an empty `CMAKE_BUILD_TYPE` (`-O0`, checked
in `rolling/core/build/*/CMakeCache.txt`), so on Rolling the server dies on every survey and the
launch file's `respawn` hides it. A one-line guard (`else if (!m_min_record.empty())`) fixes
it; with it the trial completed (3 lines, `success: true`) and gave lines identical to 1e-14 to a
`-DCMAKE_BUILD_TYPE=Release` build without the guard, i.e. to what Jazzy's GCC 13 runs today.

## Approach

Commits on `feature/issue-11` into `rolling`, one fix each, records never mixed in. [B] = cherry-picked
to `jazzy` with `-x`. Every code commit builds on Jazzy on its own.

1. **[B] CMake: replace `ament_target_dependencies` (cause 1).** `:60` becomes
   `target_link_libraries(${PROJECT_NAME} nav2_util::nav2_util_core Eigen3::Eigen)`, `:142`
   `target_link_libraries(test_control_server_lifecycle nav2_util::nav2_util_core)`. Same set as
   Jazzy's macro gave: `nav2_util_TARGETS` is exactly `nav2_util::nav2_util_core` (Jazzy's
   `nav2_util/src/CMakeLists.txt` exports only that library); `Eigen3` had no targets and gave
   its include directory, which `Eigen3::Eigen` carries. Plain signature kept, as the existing
   calls use, so the link stays transitive and the executable still gets Nav2's headers through
   the library. Exported interface after: the old list plus `nav2_util::nav2_util_core;Eigen3::Eigen`
   (checked in the scratch `export_manda_coverageExport.cmake`); no package in the project calls
   `find_package(manda_coverage)`. Alternative: `target_include_directories(... ${Eigen3_INCLUDE_DIRS})`,
   a literal copy; rejected for the target form the earlier ports use.
2. **[B] `src/SurveyPath.cpp:23` `tf2/utils.h` to `tf2/utils.hpp` (cause 2).** Jazzy's tf2 has
   `utils.hpp` (`utils.h` there is a wrapper of it), so `tf2::getYaw` is the same function.
3. **[B] Drop `src/SurveyPath.cpp:24` `#include "nav2_util/node_utils.hpp"`.** Nothing from it is
   used; it does not exist on Rolling.
4. **[B] Lifecycle node switch (causes 7/13).** In `include/manda_coverage/action_server.h`, the
   `__has_include("nav2_ros_common/lifecycle_node.hpp")` pair of includes, and in namespace
   `manda_coverage` an alias `using Nav2LifecycleNode = nav2::LifecycleNode;` /
   `nav2_util::LifecycleNode`, used as the base class and in `src/action_server.cpp:7`'s
   initialiser; `src/main.cpp:27`'s comment says "Nav2's LifecycleNode". The alias lives in the
   public header because the class is declared there; any includer gets the right type on either
   distro (in practice there is none: `action_server.cpp` is compiled into the executable and the
   test, not the library). No CMake or manifest change for `nav2_ros_common`: on Rolling
   `nav2_util` depends on it and `nav2_util::nav2_util_core` links
   `nav2_ros_common::nav2_ros_common` publicly, so its include path arrives with the target
   (verified: the scratch build has no `find_package(nav2_ros_common)`). That is what
   `ais_layer` and `bathymetry_layer` did (they name only `nav2_util`/`nav2_costmap_2d`, and
   rosdep on Jazzy never sees `nav2_ros_common`). Alternative: format 3 and
   `<depend condition="$ROS_DISTRO != jazzy">nav2_ros_common</depend>` plus a guarded
   `find_package`; rejected as larger and unlike the earlier layers.
5. **[B] Guard `RecordSwath::AddRecord`'s turn check (new, owner to confirm).** The guard above,
   plus a test in `test/test_parameters.cpp` that adds two records closer than the interval to a
   fresh `RecordSwath`; mutation-check it by removing the guard (the test aborts on Rolling at
   `-O0`). On Jazzy (GCC 13, no assertion) the test passes either way: say so in the test.
   Alternative: leave it and list it, leaving the node unusable on Rolling's default build.
6. **[B] Declare what the code uses (cause 11).** `package.xml`: `<depend>` `geometry_msgs`,
   `rcl_interfaces`, `tf2`, `tf2_geometry_msgs`, `libboost-dev` (Boost.Geometry in the public
   `PathPlan.h`); `<exec_depend>` `launch`, `launch_ros`, `lifecycle_msgs` (the launch file).
   All resolve for jazzy/noble and rolling/resolute (`rosdep resolve`, checked). Boost is the
   one a bare `ros:jazzy-ros-core` may lack; the rest arrive through declared packages today.
   No CMake change. Alternative: only `libboost-dev`; the others listed for the follow-up.
7. **[B] CI: `.github/workflows/ci.yml`** (separate commit; see CI below).
8. **Records (not backported):** `hand_run.sh` and `coverage_probe.py` under
   `.agent/work-plans/issue-11/`, the run's output, and progress entries.

**Hand run on Rolling** (package worktree's `build.sh`, `env.sh`; a `ROS_DOMAIN_ID` above 150
checked empty with `ros2 node list`): start `manda_coverage_action_server`; bring it up with
the source-built `nav2_lifecycle_manager` (`node_names:=['manda_coverage_action_server']`,
`autostart:=true`), which also exercises the bond; record `ros2 lifecycle get` and
`ros2 param list`; run `coverage_probe.py`: one goal, a 200 x 100 m rectangle rotated 30 degrees,
then a fake boat that drives each fed-back line, publishing `odom` and a flat 40 m-wide
`soundings` cloud, until the result. It prints every line and a summary. Judged without a
reference by: `success: true` within 10 lines; survey vertices inside the polygon (lead-in 15 m
and lead-out 5 m excepted); consecutive lines parallel to the first edge and spaced between
`(1 - swath_overlap) x 40` and 40 m (the trial gave 36 m); the union of the swath strips
covering most of the polygon (the trial left the far 8 m strip uncovered: the planner stops
when the next offset line falls outside, a property of the algorithm, recorded not fixed).
`hand_run.sh <setup.bash> <out_dir>` runs the same thing against any built workspace, so the owner
can run it on deadpool against Jazzy (before and after the backport) and `diff` the line files;
expected: identical within 1e-6. Not shown: the behaviour tree or `mission_manager` calling it,
Nav2's controller following the lines, real sonar geometry or TF, a real survey.

**CI (Jazzy only).** One file on both branches; triggers `push` and `pull_request` on `rolling`
and `jazzy`. One job, `ros:jazzy-ros-core`: ros-dev-tools and git; checkout; rosdep init/update;
shallow clones of rolker/marine_control and rolker/unh_marine_navigation at `jazzy`; strict
`rosdep install --ignore-src -y` (no `-r`) over `colcon list --paths-only --packages-up-to
manda_coverage`; `ament_xmllint package.xml`; `colcon build --packages-up-to manda_coverage`;
`colcon test --packages-select manda_coverage --ctest-args -LE linter` (all eight lint tests carry
the `linter` label, checked with `ctest -N -L linter`; no pytest here); a check that both gtest
result files exist; `colcon test-result --verbose`. Closure (checked over the `jazzy` clones):
manda_coverage, marine_control, marine_control_interfaces, marine_nav_interfaces,
marine_nav_utilities. No `geodesy`, and none of unh_marine_navigation's six Nav2 packages: Nav2
comes as `ros-jazzy-nav2-util` and `ros-jazzy-nav-2d-utils` binaries. The header says why there is no
Rolling job: navigation2 has no Rolling release and, by the owner's decision of 2026-11-24, no
job builds it from source, and this repository's only package needs it. Estimate 4 to 6 minutes
(unh_marine_navigation's Jazzy job with Nav2 binaries and nine packages took 4 min 49 s). Not
rehearsable here (no Jazzy, no Docker): the container, the binaries, GCC 13, the `#else` branch of
the switch, the released `nav2_util::nav2_util_core` name (`ais_layer`'s Jazzy job links the same
name). First run expected: green, 10 gtests.

**For the host, outside this branch:** rolker/unh_marine_autonomy `config/repos/core.repos` on
`rolling` pins `manda_coverage` at `jazzy`; change it to `rolling` after this merges.

## Files to Change

| File | Change |
|------|--------|
| `CMakeLists.txt` | Step 1 |
| `src/SurveyPath.cpp` | Steps 2, 3 (two include lines) |
| `include/manda_coverage/action_server.h`, `src/action_server.cpp`, `src/main.cpp` | Step 4 |
| `src/RecordSwath.cpp`, `test/test_parameters.cpp` | Step 5 |
| `package.xml` | Step 6 |
| `.github/workflows/ci.yml` (new) | Step 7 |
| `.agent/work-plans/issue-11/` | Records only |

Planner code: `PathPlan.cpp` and `lib_*` untouched; `SurveyPath.cpp` changes only includes;
`RecordSwath.cpp` gets the one guard of step 5, which is the exception to "the planner's
numerical code is not changed" and is the owner's call.

## Behaviour of the base class, Jazzy against Rolling

Read: Jazzy `nav2_util/{include,src}/lifecycle_node.*` (`jazzy`, `c7046550`) and Rolling
`nav2_ros_common/lifecycle_node.hpp`. Same: the `(name, ns, options)` constructor, the
preshutdown cleanup, `createBond`/`destroyBond`, `bond_disable_heartbeat_timeout`,
`autostart_node`, `shared_from_this()` returning the Nav2 type (used for `create_server`).
Different: `bond_heartbeat_period` defaults to 0.25 s, not 0.1 s (timeout still 4 s); two new
parameters, `warn_on_missing_params` and `strict_param_loading` (false; they govern only Nav2's
`declare_or_get_parameter`, which this package does not use); an `enable_lifecycle_services`
override is honoured; member `create_subscription`/`create_publisher`/`create_timer`/`create_service`
return Nav2 wrappers, and this package calls none of them. The trial brought the node up under
Rolling's lifecycle manager with a bond and served the action. Nothing found that changes behaviour here.

## Left alone (for the follow-up issue drafts)

Lint (copyright 34, uncrustify 34, cpplint 1,838, flake8 5; cppcheck inert on Rolling); 44
compiler warnings; unused `nav_2d_utils`; `ament_export_dependencies` missing `nav2_util`,
`marine_nav_utilities`, `rclcpp`, `nav_msgs`; `eigen` only a `build_depend` though a public
header uses it; the installed `action_server.h` declares a class no library provides;
`handle_cancel` never calls `canceled()` on the goal handle; `respawn=True` hides crashes;
"LifecycleNode is not shut down" warning on SIGINT (Rolling). In rolker/unh_marine_navigation:
`adjustPathOrientations` gives NaN quaternions for a segment along +x or -x (rotation axis zero;
seen on Rolling with an axis-aligned polygon; same code on Jazzy). Correction for #415: cause 7
does not apply to `SurveyPath.cpp`.

## Prior Art

`ais_layer` (marine_ais) and `bathymetry_layer` (unh_marine_autonomy), `rolling`: the
`__has_include` switch and naming only `nav2_util`; taken. unh_marine_navigation's and
marine_ais's workflows: the Jazzy job shape, strict rosdep over the closure, `-LE linter`; taken.
rolker/cube_bathymetry's `tile_compare.sh`: a script in the records the owner runs on Jazzy; taken.

## Serves

None (the project has no goals or design document); the Rolling port, rolker/unh_marine_autonomy#415.

## Design Picture

No change to the design picture. The project keeps no design document.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Smallest change per cause; alternatives named; the rest listed, not done |
| Test what breaks / Know whether it works | Closed-loop hand run, not only the gtests; regression test for step 5, mutation-checked |
| Verify before claiming | Jazzy is verified by CI only; what CI cannot show is listed |
| A change includes its consequences | CI added; `core.repos` pin handed to the host |
| Name the rule before bending it | Step 5 changes planner code against the port's default: flagged for the owner |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 | Yes | Plan and progress in `.agent/work-plans/issue-11/`, in commits of their own |
| Others | No | No workspace infrastructure touched |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Code on `rolling` | Same commits on `jazzy` | Hand backport by the owner |
| New CI | Both branches carry it | Yes, step 7 |
| Port done | `core.repos` pin on unh_marine_autonomy `rolling` | No: host, separate PR |
| Findings left alone | Follow-up issues (here and in unh_marine_navigation) | Drafted for the owner |

## Open Questions

- Step 5 (RecordSwath guard): include it in the port? Recommend yes: without it the node aborts
  on every survey on Rolling's default build, and the guarded result matches today's Jazzy output.
- Step 6 scope: all undeclared dependencies, or only `libboost-dev`? Recommend all (manifest only).

## Estimated Scope

One PR into `rolling` (7 code/CI commits plus records); the owner's backport PR to `jazzy`.
