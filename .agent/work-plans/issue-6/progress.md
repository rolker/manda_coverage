---
issue: 6
---

# Issue #6 — Adopt marine_control ControlServer for bridge-side operator control

## Issue Review
**Status**: complete
**When**: 2026-06-30 19:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #6
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes — single PR:
- Add `marine_control` `<depend>` to `package.xml` and a `find_package` +
  `target_link_libraries` entry to `CMakeLists.txt`.
- In `on_activate()`: construct a `marine_control::ControlServer`, then call
  `bind_parameter()` for each of the seven tuning knobs (`swath_overlap`,
  `max_bend_angle`, `swath_record_interval`, `min_allowable_swath`,
  `waypoint_distance_threshold`, `lead_in_distance`, `lead_out_distance`) with
  `units` and `group = "Coverage"`.
- In `on_deactivate()`: reset the `ControlServer` (aligns with the lifecycle note
  in `control_server.hpp`).
- No new topic handling required (ControlServer owns `~/control/state` and
  `~/control/change`).

**Right repo?** Yes — `manda_coverage` is in the project repo (`core_ws/src/`).
`marine_control` is in the same workspace. No workspace-infra changes needed.

**Dependencies?**
- #4 (declare params): already merged (PR #7 on main). ✓
- #5 (live-settable): implementation complete and pre-push review approved
  (round 3); approved for push. Dependency is substantively satisfied. ✓

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | ControlServer directly enables operator visibility and runtime adjustment of tuning knobs over the bridge — the core motivation of this issue |
| A change includes its consequences | Watch | `test_parameters.cpp` uses `rclcpp::Node` (not a LifecycleNode), so `on_activate`/`on_deactivate` lifecycle paths — including ControlServer construction/reset — are not exercised by current tests. Plan should note whether to add a lifecycle test fixture or record this as a known gap |
| Workspace vs. project separation | OK | All changes are in the `manda_coverage` project repo; `marine_control` is also a project-side package. No workspace infra changes required |
| Improve incrementally | OK | Tightly scoped Phase 3; the ControlServer API is already designed for exactly this adoption pattern |
| Only what's needed | OK | No additional abstraction. `bind_parameter()` reuses descriptors already declared by issue #4 |
| Test what breaks | Watch | See consequence note above on missing lifecycle test coverage |
| Capture decisions | OK | ADR-0003 and the ControlServer header already document the design rationale; no new ADR required |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 (Worktree isolation) | Yes — satisfied | Worktree `issue-manda_coverage-6` on `feature/issue-6` already exists |
| ADR-0003 (Workspace agnostic) | Yes — satisfied | Change is in the project repo; no workspace content affected |
| ADR-0008 (ROS 2 conventions) | Yes | `package.xml` must use `<depend>marine_control</depend>` (build + exec combined); `CMakeLists.txt` must use `find_package(marine_control REQUIRED)` and link the target |

### Consequences

- `manda_coverage` will publish `~/control/state` (ControlSet) and subscribe to
  `~/control/change` (ControlValue). Wiring these into the `udp_bridge` config is
  correctly scoped out to `unh_echoboats_project11#358`.
- The ControlServer's `on_change` callback calls `node->set_parameter()`, which
  fires the `post_set_param_callback_handle_` that acquires `m_param_mutex`
  (introduced in issue #5). This call chain is safe as long as the ControlServer's
  callback group (dedicated mutually-exclusive group, per `control_server.hpp`) and
  the post-set apply callback do not deadlock. Since `set_parameter` is called from
  the ControlServer's callback group and the post-set callback runs in the
  parameter-service group, and neither re-enters the other, no deadlock is expected.
  The implementation plan should confirm this with a brief analysis.

### Actions
- [ ] Plan should address lifecycle test coverage: either add a test fixture using
  a LifecycleNode to exercise `on_activate`/`on_deactivate` (including
  ControlServer construction/reset), or explicitly record this as a known gap with
  a follow-up issue.
- [ ] Implementation should confirm the `set_parameter` → post-set callback call
  chain (ControlServer's `on_change` → `m_param_mutex`) is deadlock-free and add
  a brief comment if the analysis is non-obvious.

## Plan Authored
**Status**: complete
**When**: 2026-06-30 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-6/plan.md` at `6be6a44`
**Branch**: feature/issue-6 at `6be6a44`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-30 18:30 +00:00
**By**: Claude Code Agent (Claude Opus)

<!-- Independent review: the Plan Authored entry shares the workspace agent name
"Claude Code Agent", but this review is a fresh-context, host-dispatched
sub-agent on a different model (Sonnet authored, Opus reviewing). The name-match
self-review heuristic misfires on the shared name; this is a genuinely
independent review, so no self-review annotation. -->

**Plan**: `.agent/work-plans/issue-6/plan.md` at `6be6a44`
**PR**: PR-less (--issue mode; reviewed in worktree `issue-manda_coverage-6`)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) review-issue Action #1 (lifecycle test coverage) not carried into the plan — adoption path (`on_activate` construct+bind, `on_deactivate` reset) is exercised by nothing; `test_parameters.cpp` uses `rclcpp::Node`, not the LifecycleNode. Add a lifecycle fixture or record a known gap + follow-up issue. — `plan.md:60`
- [ ] (suggestion) review-issue Action #2 (deadlock analysis: `ControlServer::on_change` → `set_parameter` → post-set `m_param_mutex`) dropped; Open Questions says "None". Carry the check forward as an implementation step. — `plan.md:84`
- [ ] (suggestion) Teardown wired only in `on_deactivate()`; a direct active→shutdown skips it and leaves `control_server_` destruction to the node destructor — `control_server.hpp:38` warns against destroying while spinning. Also reset in `on_cleanup`/`on_shutdown`, or confirm deactivate always precedes shutdown. — `plan.md:45`

## Implementation
**Status**: complete
**When**: 2026-06-30 18:50 +00:00
**By**: Claude Opus

**Branch**: feature/issue-6
**Commits**:
- `4cf7eee` build: depend on marine_control for the action server
- `3f18432` feat: adopt marine_control ControlServer for operator control
- `2ad97a0` test: lifecycle test for the ControlServer adoption
- (this entry) plan sync + progress

All three Plan Review suggestions were folded in.

### What was built
- **Dependency wiring** — `package.xml` `<depend>marine_control</depend>`;
  `CMakeLists.txt` `find_package(marine_control REQUIRED)` and
  `marine_control::marine_control` linked into `manda_coverage_action_server`.
  (Also dropped a stray trailing space on the `add_executable` line, which was
  the sole `ament_lint_cmake` error — that linter now passes.)
- **Member** — `action_server.h` includes `marine_control/control_server.hpp`
  and adds `std::unique_ptr<marine_control::ControlServer> control_server_`.
- **Construct + bind** — `on_activate()` constructs the server with
  `opts.device_name = "Manda Coverage"` and `bind_parameter()`s all seven knobs
  with units + `group = "Coverage"`.
- **Suggestion 1 (MUST-FIX) — reset in all teardown paths** —
  `control_server_.reset()` added before the existing `survey_path_` teardown in
  `on_deactivate()`, `on_cleanup()`, and `on_shutdown()`. `reset()` is idempotent
  on a null `unique_ptr`, so each path is safe when already null (verified by
  reading: deactivate→cleanup leaves it null at cleanup; a configured-but-never-
  activated node has it null at cleanup/shutdown; active→shutdown resets exactly
  once in shutdown).
- **Suggestion 3 — deadlock analysis** — documented as a comment by the
  construction in `on_activate()` and in `plan.md`: `on_change` (server's own
  mutually-exclusive callback group) → `set_parameter` → post-set apply
  (parameter-service group, `m_param_mutex`); neither re-enters the other, no
  lock-order inversion.
- **Suggestion 2 — lifecycle test** — `test/test_control_server_lifecycle.cpp`
  drives configure→activate, asserts a `ControlSet` heartbeat on
  `~/control/state` with the 7 bound knobs (`group=="Coverage"`), then
  deactivate and asserts the heartbeat stops. Bounded `spin_some` + steady-clock
  deadline poll (no fixed sleeps). Context init is in `SetUpTestSuite` because the
  fixture's executor member is constructed before `SetUp()`.
- **Plan sync** — fixed the Issue URL to
  `https://github.com/rolker/manda_coverage/issues/6`; reflected the all-paths
  reset, the deadlock note, and the new test in Approach / Files-to-Change /
  Principles / Open Questions.

### Build & test
- `colcon build --packages-up-to manda_coverage` — clean (pre-existing
  unused-parameter warnings in legacy code only).
- gtest: **all 10 cases pass** — `test_parameters` (9) + `test_control_server_lifecycle` (1).
- **Pre-existing legacy lint failures remain** (`ament_copyright`,
  `ament_cpplint`, `ament_uncrustify`, `ament_flake8` — ~1900 findings across ~38
  unchanged MOOS-derived files: `lib_geometry/*`, `lib_mbutil/*`, `PathPlan.h`,
  `RecordSwath.h`, `SurveyPath.h`, `launch/*.py`, and the never-headered
  `action_server.{h,cpp}`). These are unrelated to #6 and were red before this
  work; my diff touches only 5 files and adds **zero** new lint findings (the new
  test file and CMake edits are lint-clean; `ament_lint_cmake` was fixed). Not
  reformatting the legacy library is in keeping with "Only what's needed".

### Next step
Ready for review-code. No push / PR (host publishes after local review). PR will
carry `Closes #6`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 18:55 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-6 at `88e9578`
**Mode**: pre-push
**Depth**: Deep (reason: ≥200 changed lines + concurrency/lifecycle-relevant change)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — a genuine concurrency-correctness concern (race under the production MultiThreadedExecutor) warrants address-findings + re-review before push.

### Findings
- [x] (must-fix) ControlServer construct/bind (`on_activate`) and reset (`on_deactivate`/`on_cleanup`/`on_shutdown`) run while the node spins on a `MultiThreadedExecutor` (`main.cpp:17`), violating `control_server.hpp`'s "bind before spinning / destroy only when not spinning" contract — the server's own MutuallyExclusive callback group can run on a different thread than the lifecycle transitions, racing the unsynchronized `bindings_` map during bind and racing timer/sub teardown against an in-flight heartbeat during reset (UB as shipped). Fix e.g. via SingleThreadedExecutor or shared callback group. — `src/action_server.cpp:55-64,76,88,101`
- [x] (suggestion) Deadlock-analysis comment misdescribes the mechanism: the post-set apply (`add_post_set_parameters_callback`, `SurveyPath.cpp:162`) runs synchronously inline on `on_change`'s thread, not a separate "parameter-service group"; conclusion (no deadlock) holds but the comment asserts non-existent thread-separation and omits the real bind/teardown race. — `src/action_server.cpp:44-54`
- [x] (suggestion) Lifecycle test uses SingleThreadedExecutor and transitions before `add_node`, so it cannot reproduce the production MTExecutor race or the deactivate→activate re-bind path; add an MT-executor / re-activation variant. — `test/test_control_server_lifecycle.cpp:90-94`
- [x] (suggestion) Single `spin_some()` before snapshotting `count_at_deactivate` may not drain a RELIABLE heartbeat in transit (minor flakiness; errs toward false failure, not false pass). — `test/test_control_server_lifecycle.cpp:127-138`

### Operator decision (round 1, 2026-06-30)
Fix the must-fix via **SingleThreadedExecutor in manda** and address all 3 suggestions:
- **(must-fix) Switch `src/main.cpp` to `rclcpp::executors::SingleThreadedExecutor`.**
  A single executor thread means lifecycle transitions (construct/bind in
  `on_activate`, `reset()` in `on_deactivate`/`on_cleanup`/`on_shutdown`) can never
  run concurrently with the ControlServer's own timer/change-sub callbacks, so the
  `bindings_` bind race and the timer/sub-teardown-vs-heartbeat race are both
  eliminated — satisfying `control_server.hpp`'s no-concurrent-execution contract.
  **Keep** `m_param_mutex` from #5 (defensive; documents the param-apply-vs-planning
  invariant). **Update the now-stale comments** that cite a "MultiThreadedExecutor"
  as the mutex rationale — `src/SurveyPath.cpp` (~lines 161, 222) and the
  `m_param_mutex` member comment in `include/manda_coverage/SurveyPath.h`: state that
  the node now uses a SingleThreadedExecutor so callbacks are already serialized, and
  the mutex is retained defensively to guard the param-apply-vs-planning invariant
  should the executor change.
- **(suggestion) Fix the deadlock-analysis comment** in `src/action_server.cpp`: the
  post-set apply runs **synchronously inline on `on_change`'s thread** (not a separate
  "parameter-service group"). State the accurate mechanism — and that with the
  SingleThreadedExecutor there is no concurrency between the lifecycle bind/reset and
  the server's callbacks.
- **(suggestion) Test: add the deactivate→activate re-bind path.** Extend
  `test/test_control_server_lifecycle.cpp` to deactivate then **re-activate** and
  assert the `ControlSet` heartbeat resumes with the 7 bound items. **Keep the test on
  a SingleThreadedExecutor** to match production (drop the MT-executor variant — prod
  is now single-threaded).
- **(suggestion) Fix the heartbeat drain** before snapshotting `count_at_deactivate`:
  use a bounded `spin_some` poll loop (with a short deadline) to drain any in-flight
  RELIABLE heartbeat before recording the baseline, rather than a single `spin_some`.
- **Plan + follow-up**: update `plan.md` to note the SingleThreadedExecutor choice.
  The HOST will file a `marine_control` follow-up issue documenting the MT-adoption
  gap (the header recommends `on_activate` construction but the server isn't MT-safe
  while spinning) — address-findings need not file it.
Re-run the package build + tests (expect all pass); host re-dispatches review-code.

## Implementation
**Status**: complete
**When**: 2026-06-30 19:11 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-6 at `a5f92c8`
**Addressed**: Local Review (Pre-Push) (2026-06-30 18:55 +00:00, branch `feature/issue-6` at `88e9578`)
**Commits**:
- `b21d767` fix: serialize ControlServer lifecycle via SingleThreadedExecutor (#6)
- `f765a4d` docs: correct ControlServer deadlock-analysis comment (#6)
- `ac2a1b2` test: drain in-flight heartbeat before deactivate baseline (#6)
- `8af1447` test: exercise deactivate->activate ControlServer re-bind path (#6)
- `a5f92c8` docs: sync plan with SingleThreadedExecutor + re-bind test (#6)
- (this entry) progress

Addressed all four findings from the pre-push review, following the operator
decision (SingleThreadedExecutor in manda; address all three suggestions).

### Actions
- [x] (must-fix) ControlServer bind/reset raced the server's own callbacks under the
  `MultiThreadedExecutor` — `src/main.cpp` now spins on a
  `rclcpp::executors::SingleThreadedExecutor`, so lifecycle transitions
  (construct/bind in `on_activate`, `reset()` in `on_deactivate`/`on_cleanup`/
  `on_shutdown`) can never run concurrently with the server's timer/change-sub
  callbacks. The `bindings_` bind race and the timer/sub-teardown-vs-heartbeat race
  are both structurally eliminated. `m_param_mutex` (#5) is **kept** defensively;
  the now-stale "MultiThreadedExecutor" rationale comments in
  `include/manda_coverage/SurveyPath.h` and `src/SurveyPath.cpp` (the post-set apply
  and `set_goal` notes) were updated to state the node is now single-threaded and
  the mutex guards the param-apply-vs-planning invariant. — `src/main.cpp:16`,
  `src/SurveyPath.cpp:156-161,217-223`, `include/manda_coverage/SurveyPath.h:133-141`
- [x] (suggestion) Deadlock-analysis comment corrected — `src/action_server.cpp` now
  states the post-set apply runs **synchronously inline on `on_change`'s thread**
  (rclcpp fires post-set callbacks inside `set_parameter`, not via a separate
  "parameter-service group"), the conclusion (no deadlock, single lock held) holds,
  and notes the SingleThreadedExecutor removes lifecycle/callback concurrency. — `src/action_server.cpp:45-56`
- [x] (suggestion) Lifecycle test now exercises the deactivate→activate re-bind path —
  after the deactivate "heartbeat stops" check, the test re-activates and asserts the
  `ControlSet` heartbeat resumes with the seven re-bound knobs (`group=="Coverage"`).
  Kept on a `SingleThreadedExecutor` to match production (no MT variant — prod is now
  single-threaded). — `test/test_control_server_lifecycle.cpp:156-181`
- [x] (suggestion) Heartbeat drain fixed — the pre-deactivate baseline
  (`count_at_deactivate`) is now captured only after a bounded `spin_some` quiescence
  loop (settle window + 2s deadline), so an in-flight RELIABLE heartbeat cannot read as
  a spurious "heartbeat continued" later. — `test/test_control_server_lifecycle.cpp:135-149`

### Build & test
- `colcon build --packages-up-to manda_coverage` — clean (only pre-existing
  unused-parameter warnings in legacy code).
- gtest: **all 10 cases pass** — `test_parameters` (9) + `test_control_server_lifecycle`
  (1, now configure→activate→deactivate→**re-activate**).
- Pre-existing legacy lint failures unchanged (`ament_copyright`, `ament_cpplint`,
  `ament_flake8`, `ament_uncrustify` on the ~34 MOOS-derived files + never-headered
  `action_server`/`main`). My diff adds **zero** new lint findings: the test file is
  fully lint-clean (uncrustify/cpplint/copyright), and `main.cpp`'s only uncrustify
  divergence is the pre-existing trailing blank line (the file was already red on
  `ament_copyright`). Not reformatting legacy is in keeping with "Only what's needed".

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). No push / PR
(host publishes after local review). Re-review reads the diff cold and confirms the
race fix and the three suggestion fixes are genuinely resolved.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 19:23 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-6 at `c442784`
**Mode**: pre-push
**Depth**: Deep (reason: ≥200 changed lines + concurrency/lifecycle change)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 2 | **Ship**: recommended — round-1 must-fix (MTExecutor bind/teardown race on the normal transition path) genuinely resolved; 0 new must-fix; residual items are a known marine_control adoption-gap + test/comment polish.

### Findings
- [x] (suggestion) main.cpp/action_server.cpp threading comments overstate the guarantee: SingleThreadedExecutor serializes lifecycle-manager-driven transitions (executor thread) but nav2_util's rcl pre-shutdown callback drives deactivate/cleanup -> control_server_.reset() on the signal-handler thread, so the timer/sub-teardown-vs-heartbeat race persists on the SIGINT-while-active path; scope the comment + confirm the marine_control adoption-gap follow-up is filed — `src/main.cpp:16-25`, `src/action_server.cpp:54-56`
- [x] (suggestion) SingleThreadedExecutor responsiveness tradeoff: a long planning callback (odom/ping -> CreateNewPath -> PathPlan) now blocks the ControlServer heartbeat/change handling and action handling on the one thread; deliberate round-1 decision, worth recording — `src/main.cpp:25`
- [x] (suggestion) test TearDown never remove_node(node_) (only sub_node_); node_.reset() drops the owner while still registered — benign now, fragile if TearDown ever spins — `test/test_control_server_lifecycle.cpp:74-80`
- [x] (suggestion) test ends in active state; final control_server_ teardown runs from the unique_ptr destructor, not on_deactivate/on_shutdown (on_deactivate is still exercised mid-test) — `test/test_control_server_lifecycle.cpp:74-80`

### Next step
Lifecycle: **Local Review (approved)** → host publishes (push / open PR with `Closes #6`) → **triage-reviews**. Verdict is approved (0 must-fix); the 4 suggestions are non-blocking — apply the comment-scope fix opportunistically or carry forward. Static analysis clean on changed lines; pre-existing legacy MOOS lint debt untouched and out of scope.

### Operator decision (round 2, 2026-06-30)
Address all four suggestions, then re-review:
- **(1) Scope the threading comment honestly.** In `src/main.cpp` (and the
  `on_deactivate`/reset note in `src/action_server.cpp`), do NOT claim the
  bind/teardown race is fully eliminated. State accurately: the
  `SingleThreadedExecutor` serializes the **lifecycle-manager-driven** transitions
  (they run on the executor thread, so they can't race the server's timer/sub),
  **but** `nav2_util::LifecycleNode` registers an rcl pre-shutdown callback that
  drives `deactivate`/`cleanup` → `control_server_.reset()` on the **signal-handler
  thread** during SIGINT — a one-shot teardown-vs-heartbeat window at process exit
  that single-threading does not close. Reference the general fix:
  `rolker/marine_control`#12 (inject an external callback group / lock `bindings_`).
- **(2) Record the responsiveness tradeoff.** Add a brief comment (near the
  executor in `main.cpp`) noting the deliberate round-1 decision: single-threaded
  means a long planning callback (odom/ping → CreateNewPath → PathPlan) blocks the
  ControlServer heartbeat/change handling and action handling on the one thread;
  acceptable for this lightweight planner.
- **(3) test TearDown.** `remove_node(node_)` in `TearDown` (it currently removes
  only `sub_node_`) before `node_.reset()`, so the executor doesn't hold a node
  that's being dropped.
- **(4) test end state.** End the test by driving `on_deactivate` (and
  `on_shutdown` if natural) so the final `control_server_` teardown runs through the
  lifecycle path rather than only the `unique_ptr` destructor — exercising the
  teardown path we actually ship.
Re-run the package build + tests (expect all 10 pass); host re-dispatches review-code.
The `marine_control` follow-up (#12) is already filed by the host.

## Implementation
**Status**: complete
**When**: 2026-06-30 19:45 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-6 at `33a1f55`
**Addressed**: Local Review (Pre-Push) (2026-06-30 19:23 +00:00, branch `feature/issue-6` at `c442784`) — round 2, approved, 4 suggestions
**Commits**:
- `94e6976` docs: scope ControlServer threading comments for SIGINT teardown race (#6)
- `7a1d33c` docs: record SingleThreadedExecutor responsiveness tradeoff (#6)
- `d3b5ec1` test: remove lifecycle node from executor before reset in TearDown (#6)
- `1eaf23c` test: end lifecycle test via deactivate/shutdown teardown path (#6)
- `33a1f55` test: keep final-deactivate drain loop uncrustify-clean (#6)
- (this entry) progress

Addressed all four round-2 suggestions following the operator decision (address
all four, then re-review). The verdict was already **approved** (0 must-fix); these
are non-blocking polish on an approved diff.

### Actions
- [x] (suggestion) Threading comments overstated the guarantee — `src/main.cpp` and
  the `on_activate` deadlock note + `on_deactivate` reset note in
  `src/action_server.cpp` now state honestly that the `SingleThreadedExecutor`
  serializes only the **lifecycle-manager-driven** transitions (executor thread),
  while nav2_util's rcl pre-shutdown callback drives `deactivate`/`cleanup` →
  `control_server_.reset()` on the **signal-handler thread** during SIGINT, leaving a
  one-shot teardown-vs-heartbeat window at process exit. Each comment references the
  general fix `rolker/marine_control#12` (already filed by the host). — `src/main.cpp:16-32`, `src/action_server.cpp:54-57,76-81`
- [x] (suggestion) Responsiveness tradeoff recorded — a comment by the executor in
  `src/main.cpp` notes the deliberate round-1 decision: single-threaded means a long
  planning callback (odom/ping → CreateNewPath → PathPlan) blocks the ControlServer
  heartbeat/change and action handling until it returns; acceptable for this
  lightweight planner, revisit with the marine_control#12 fix if planning latency
  starves operator control. — `src/main.cpp:34-40`
- [x] (suggestion) Test `TearDown` now `remove_node(node_->get_node_base_interface())`
  before `node_.reset()`, so the executor never holds the lifecycle node while it is
  being destroyed (no-op if a path left it un-added). — `test/test_control_server_lifecycle.cpp:74-85`
- [x] (suggestion) Test now ends by driving the shipped lifecycle teardown rather than
  the `unique_ptr` destructor: after the re-bind cycle it `deactivate()`s (drains
  in-flight heartbeats, asserts the heartbeat stops), then `shutdown()`s to the
  `finalized` state — exercising the `on_deactivate`/`on_shutdown` reset paths
  end-to-end. — `test/test_control_server_lifecycle.cpp:181-205`

### Build & test
- `colcon build --packages-up-to manda_coverage` — clean (only pre-existing
  unused-parameter warnings in legacy code).
- gtest: **all 10 cases pass** — `test_parameters` (9) + `test_control_server_lifecycle`
  (1, now configure→activate→deactivate→re-activate→**deactivate→shutdown**).
- **Zero new lint findings**: the test file is fully clean
  (`ament_uncrustify`/`ament_cpplint`/`ament_copyright` all "No problems found" — the
  wrapped drain-loop condition that uncrustify reflowed was collapsed back to a single
  line in `33a1f55`); the `main.cpp`/`action_server.cpp` edits are comment-only and add
  no divergences (`main.cpp`'s sole uncrustify item remains the pre-existing trailing
  blank line). Pre-existing legacy MOOS lint debt (`ament_copyright`, `ament_cpplint`,
  `ament_flake8`, `ament_uncrustify` on the unchanged MOOS-derived files) is untouched
  and out of scope.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). No push / PR
(host publishes after local review). The re-review reads the diff cold and confirms
the four suggestions are genuinely resolved.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 21:41 +00:00
**By**: Claude Opus
**Verdict**: approved

**Branch**: feature/issue-6 at `a2fc991`
**Mode**: pre-push
**Depth**: Deep (reason: concurrency/lifecycle change + ≥200 changed lines)
**Must-fix**: 0 | **Suggestions**: 1
**Round**: 3 | **Ship**: recommended — round-1 must-fix (MTExecutor bind/teardown race) remains structurally resolved by the SingleThreadedExecutor; all four round-2 suggestions genuinely applied; 0 new must-fix; the sole suggestion is pre-existing and out of scope.

### Findings
- [ ] (suggestion) Pre-existing, out of scope: `action_server_` (created in `on_configure`) is not reset in `on_cleanup`/`on_shutdown`, unlike `survey_path_`/`control_server_`; a cleanup→reconfigure cycle would re-create_server on the same name. Predates this PR; noted only because this change establishes the reset-in-every-teardown discipline. Consider a separate issue. — `src/action_server.cpp:21,92-117`

### Round-2 fix verification
- [x] Threading comments scoped honestly (SingleThreadedExecutor serializes only the lifecycle-manager-driven path; SIGINT-while-active reset() still on the signal-handler thread → `marine_control#12`) — `src/main.cpp:25-32`, `src/action_server.cpp:54-59,81-84`
- [x] Responsiveness tradeoff recorded — `src/main.cpp:35-39`
- [x] Test `TearDown` removes `node_` from the executor before reset — `test/test_control_server_lifecycle.cpp:80`
- [x] Test ends via the shipped lifecycle teardown (deactivate→shutdown), not the unique_ptr destructor — `test/test_control_server_lifecycle.cpp:191-208`

### Specialist notes
- Static analysis: new test file fully clean (uncrustify/cpplint/copyright); CMakeLists clean; package.xml well-formed. Changed source files carry only pre-existing legacy uncrustify debt — verified the `origin/jazzy` base of `action_server.h` diverges identically, so the new `control_server_` member adds no new finding. Zero new lint on changed lines.
- Governance: ADR-0003 (canonical D4/D5/D6 impl) and ADR-0008 satisfied; udp_bridge wiring correctly deferred to #358. Round-1 "test what breaks" Watch closed by the lifecycle test.
- Plan drift: zero — implementation matches the plan's Files-to-Change and all six approach steps.
- Adversarial (2 disjoint passes, no must-fix): Lens A — 7 bound names exactly match the 7 `declare_bounded` params + the post-set apply switch; re-activate path sound (WeakPtr group expiry, auto-add new group). Lens B — no CMake ODR (action_server.cpp not in the library, only executable+test); deadlock comment accurate (post-set fires inline inside set_parameters); SIGINT residual honestly documented + tracked (#12); bond heartbeat unaffected by single-threading.

### Next step
Lifecycle: **Local Review (approved)** → host publishes (push / open PR with `Closes #6`) → **triage-reviews**. Verdict is approved (0 must-fix); the single suggestion is pre-existing/out-of-scope and need not block the push. Static analysis clean on changed lines; legacy MOOS lint debt untouched and out of scope.
