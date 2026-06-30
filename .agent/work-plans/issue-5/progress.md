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
