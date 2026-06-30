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
