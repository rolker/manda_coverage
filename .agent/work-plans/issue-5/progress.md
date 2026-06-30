---
issue: 5
---

# Issue #5 — Make coverage-planner parameters live-settable at runtime

## Issue Review
**Status**: complete
**When**: 2026-06-30 17:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #5
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes — the issue defines a clear deliverable (register
`add_on_set_parameters_callback`, validate against descriptor ranges, update
cached members + live PathPlan/RecordSwath state), names each parameter
explicitly, marks `soundings_topic`/`display_topic` as read-only, and
requires tests for both the accept and reject paths. Fits a single PR.

**Right repo?** Yes — `SurveyPath`, `PathPlan`, and `RecordSwath` all live
in the `manda_coverage` project repo. No workspace-infra changes required.

**Dependencies?** Depends on #4 (coverage-density knobs as declared
parameters). #4 is already merged (PR #7 landed on main), so this
dependency is satisfied and work can begin immediately.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Safety First (project) | Watch | The on-set callback is the gatekeeper for Phase 3 marine_control bridge-side control; validation must reject out-of-range values with populated `reason` strings, not just silently clamp |
| Human control and transparency | OK | Live-settable params directly improve operator control; `read_only` on topic params makes restart-only semantics explicit |
| A change includes its consequences | OK | Tests are in scope; the issue identifies `RecordSwath` interval fixed at construction and requires it to be made live |
| Test what breaks | Watch | Tests must verify that the live PathPlan/RecordSwath state is updated (not just the cached member) and that out-of-range values leave state unchanged |
| Only what's needed | OK | Scope is bounded to the named parameters; no speculative additions |
| Improve incrementally | OK | Phase 2 of a phased rollout; single-PR scope |
| Modularity and Decoupling (project) | Watch | If setters are added to `RecordSwath`/`PathPlan`, they should remain general-purpose; the plan should document the design choice (setter vs re-read per cycle) |
| Standards Compliance (project) | OK | `add_on_set_parameters_callback` returning `SetParametersResult` is the standard ROS 2 pattern |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0008 (ROS 2 conventions) | Yes | Callback must return `rcl_interfaces::msg::SetParametersResult`; confirm lifecycle-node parameter-callback registration order (`on_configure` vs constructor) |
| ADR-0002 (worktree isolation) | Satisfied | Worktree `issue-manda_coverage-5` on `feature/issue-5` already exists |

### Consequences

- **RecordSwath**: `SetInterval()` is currently called once in `configure()` (SurveyPath.cpp:129); must become live — either via a new setter or by re-reading the cached value each planning cycle. Plan should document the choice.
- **`soundings_topic` / `display_topic`**: Marking `read_only` breaks callers that attempt runtime `ros2 param set` on these — intentional, but worth calling out in the PR description.
- **test_parameters.cpp**: Already exists; extend it rather than add a parallel test file.

### Actions
- [ ] Plan must document the design decision: setter vs re-read per cycle for `RecordSwath.SetInterval`.
- [ ] Ensure validation returns populated `SetParametersResult.reason` strings (required for Phase 3 marine_control safety gating), not silent accepts/rejects.
- [ ] Tests must assert live PathPlan/RecordSwath state update — not just cached member — after an in-range set.

## Plan Authored
**Status**: complete
**When**: 2026-06-30 18:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-5/plan.md` at `3e84ec9`
**Branch**: feature/issue-5 at `3e84ec9`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-30 16:58 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-5/plan.md` at `3e84ec9`
**PR**: PR-less (local `feature/issue-5`; no draft PR)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) Step 3's callback contract is self-contradictory — "`successful = true` ... on any failure" — clarify: return `successful=true` on accept, `successful=false` with `reason` on type mismatch; descriptor-range rejections are produced by rclcpp *before* the callback fires, so the test's `result.successful==false` for out-of-range comes from ROS, not the callback — `plan.md:50`
- [ ] (suggestion) ROS conventions: mutating cached members inside the on-set (pre-set) callback applies state before the set is finalized; an atomic multi-param `set_parameters` where a later param is rejected leaves earlier mutations un-rolled-back. Idiomatic Jazzy/Rolling splits validate (on-set) from apply (`add_post_set_parameters_callback`). Acceptable to keep the single on-set callback for this node's single-knob tuning, but note the limitation — `plan.md:43`
- [ ] (suggestion) Consequences table lists a descriptor only for `waypoint_distance_threshold_`; step 2 also adds `[0,∞)` descriptors to `lead_in_distance`/`lead_out_distance` — add them to the table for completeness (same benign effect: rejects nonsensical negatives) — `plan.md:108`
- [ ] (suggestion) New `SurveyPathLiveParamTest` overlaps the existing `InRangeAccepted`/`OutOfRangeRejection` tests (`test_parameters.cpp:74`); ensure it adds the *state-assertion via the new getters* those lack rather than duplicating accept/reject coverage — `plan.md:72`

## Implementation
**Status**: complete
**When**: 2026-06-30 17:20 +00:00
**By**: Claude Opus

**Branch**: feature/issue-5 (4 commits: 0f2878d, b359261, 8731c01, 9c135ba)
**PR**: none (local-first; host publishes after local review)

### What was done
Implemented the plan with the operator-chosen validate/apply split and all
three Plan Review cleanups:

- `RecordSwath.h`: added `GetMinAllowableSwath() const`; made `IntervalDist()`
  const so the new `SurveyPath` getters can delegate from a const context.
- `SurveyPath.cpp`:
  - `soundings_topic`/`display_topic` declared `read_only` via a new
    `declare_read_only_string` lambda (ROS rejects sets before any callback).
  - The three distance params (`waypoint_distance_threshold`,
    `lead_in_distance`, `lead_out_distance`) now use `declare_bounded` with
    `[0, max]` ranges (lambda moved above their declarations).
  - **on-set callback = VALIDATE only**: returns `SetParametersResult`,
    `successful=false` with a populated `reason` on a double type mismatch;
    mutates no state. Descriptor range violations are rejected by rclcpp first.
  - **post-set callback = APPLY**: writes cached members
    (`m_swath_overlap`, `m_max_bend_angle`, distance members) and calls live
    `RecordSwath` setters (`SetInterval`, `SetMinAllowableSwath`) only after the
    set commits — atomic-safe for multi-param sets.
  - Both handles reset in `cleanup()`.
- `SurveyPath.h`: added `on_set_param_callback_handle_` and
  `post_set_param_callback_handle_` members and four const test getters
  (`swath_overlap`, `max_bend_angle`, `swath_record_interval`,
  `min_allowable_swath`).
- `test_parameters.cpp`: added `InRangeSetUpdatesLiveState` (asserts getters
  reflect new values after an accepted set — exercises the post-set apply path)
  and `RejectedSetLeavesLiveStateUnchanged` (rejected set + state unchanged).
  These add the state-assertion coverage the existing accept/reject tests lack.
- `plan.md`: steps 3-4 reworded for the validate/apply split + two handles +
  cleanup reset; consequences table now lists all three distance descriptors
  plus the split's cleanup requirement.

### Plan Review findings — all addressed
- [x] Callback-contract contradiction — resolved by the validate/apply split
  (on-set never claims success on failure).
- [x] Atomic multi-param concern — adopted the post-set apply split.
- [x] Consequences table — added `lead_in_distance`/`lead_out_distance` rows.
- [x] Test overlap — new tests assert state via getters, not duplicate
  accept/reject coverage.

### Build & test
- `colcon build --packages-up-to manda_coverage`: success (deps
  marine_nav_interfaces/marine_nav_utilities built first; only pre-existing
  unused-parameter warnings).
- `colcon test --packages-select manda_coverage`: gtest **9/9 pass, 0 failures**
  (7 prior + 2 new; new tests confirmed `status="run"`). The remaining
  `colcon test-result` failures are pre-existing ament_lint debt (copyright,
  cpplint, uncrustify, flake8, lint_cmake) on the legacy MOOS-derived code —
  verified identical to the baseline with my changes stashed, so this PR
  introduces no new lint failures. Issue #4 merged through the same lint state.

### Next step
Host local review (review-code), then publish with `Closes #5`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 17:22 +00:00
**By**: Claude Code Agent (Claude Opus 4.6)
**Verdict**: approved

**Branch**: feature/issue-5 at `3f50f92`
**Mode**: pre-push
**Depth**: Standard (reason: ~195 LOC across 4 C++ files + project plan.md override-trigger; no Deep promotion triggers)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended — no must-fix; builds, 9/9 tests pass, plan faithfully implemented; suggestions can be applied in a quick pass or tracked.

### Findings
- [ ] (suggestion) Data race: post-set callback (param-service default group) writes m_swath_overlap/m_max_bend_angle/distance members/m_swath_record concurrently with ping/odom group reads under MultiThreadedExecutor — benign (independent aligned scalar doubles) but formal UB; use atomic/mutex/shared group or document — `src/SurveyPath.cpp:178`
- [ ] (suggestion) On-set VALIDATE callback type-check is unreachable (statically-typed doubles → rclcpp rejects type mismatch before callback); the Phase-3 `reason` it claims to produce never fires — drop or strengthen — `src/SurveyPath.cpp:156`
- [ ] (suggestion) Three live params (waypoint_distance_threshold, lead_in_distance, lead_out_distance) have no getter/apply-path test; a wrong-member copy-paste would pass all tests — add getters + assertions — `test/test_parameters.cpp:110`
- [ ] (suggestion) Range-rejection `reason` is rclcpp-generic, not operator-tailored, despite the design comment implying operator-facing diagnostics — `src/SurveyPath.cpp:152`

### Notes
- Static analysis: cppcheck clean; `cpplint` not installed on this host (C++ style unchecked).
- No project-level PRINCIPLES/ADRs in repo; ADR-0008 (ROS 2 conventions) satisfied — idiomatic validate/apply split returning SetParametersResult.

### Operator decision (publish checkpoint, 2026-06-30)
Address all four suggestions before publishing, per these per-finding decisions:
- **(1) Data race — FIX it, do not just document.** Ensure the post-set apply
  path never runs concurrently with the odom/ping planning readers of the shared
  members (`m_swath_overlap`, `m_max_bend_angle`, the distance members, and
  `m_swath_record` via its setters). Pick the correct, idiomatic Jazzy mechanism
  and **verify it actually serializes the two paths**: either (a) a `std::mutex`
  guarding the shared members in BOTH the apply callback and the odom/ping
  readers, or (b) placing the relevant subscriptions and the parameter-apply in a
  single `MutuallyExclusive` callback group. Note: parameter callbacks may not
  honor subscription callback groups — if (b) does not genuinely serialize against
  the planning reads, use (a) the mutex. Confirm the chosen mechanism also covers
  `m_swath_record`'s setter-mutated state.
- **(2 + 4) DROP the vestigial on-set validate callback.** rclcpp validates type
  and descriptor range before any on-set callback fires for these statically-typed
  doubles, so the on-set type-check is dead code and its Phase-3 `reason` never
  fires. Remove `add_on_set_parameters_callback` and its handle; keep ONLY the
  `add_post_set_parameters_callback` apply path (that is where the atomicity
  benefit lives). Add a brief comment documenting that range/type rejection (with
  its rclcpp-generic `reason`) is handled by the descriptors, not a callback —
  which also resolves #4 (operator-tailored reasons for range violations are not
  achievable via callbacks since rclcpp rejects first).
- **(3) Close the test gap.** Add const getters for `waypoint_distance_threshold`,
  `lead_in_distance`, `lead_out_distance` and apply-path assertions: after an
  in-range `set_parameter` on each, assert the corresponding getter reflects the
  new value (so a wrong-member copy-paste in the apply callback would fail).
Re-run the package build + tests (expect all to pass); then the host
re-dispatches review-code.
