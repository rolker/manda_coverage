---
issue: 4
---

# Issue #4 — Expose coverage-density tuning constants as declared parameters

## Issue Review
**Status**: complete
**When**: 2026-06-30 15:10 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #4
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes — Phase 1 is explicitly bounded to launch-time configurability only;
live/runtime settability is deferred to Phase 2 (`rolker/unh_echoboats_project11#358`).
All changes are within a single package (`manda_coverage`) and can land in a single PR.

**Right repo?** Yes — `SurveyPath`, `PathPlan`, and `RecordSwath` all live in the
`manda_coverage` project repo. No workspace-infra changes required.

**Dependencies?** Parent tracker is `rolker/unh_echoboats_project11#358`, but this Phase 1
work is self-contained; it does not depend on any other open issue.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Promotes hardcoded knobs to operator-visible ROS 2 parameters with bounds descriptors — direct improvement to configurability |
| A change includes its consequences | OK | Issue explicitly includes tests (defaults + out-of-range rejection) and retires the dead `m_swath_interval` member |
| Only what's needed | OK | Phase 1 scope is minimal and correct — no runtime settability, no extra abstraction |
| Improve incrementally | OK | Single package, single PR, explicit Phase 1 of a two-phase plan |
| Test what breaks | OK | Tests target the declaration behavior (defaults and range rejection), which is the risky logic here |
| Workspace vs. project separation | OK | All changes are in `manda_coverage`; workspace infra untouched |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Already satisfied — `issue-manda_coverage-4` worktree exists on `feature/issue-4` |
| ADR-0008 — Follow ROS 2 conventions | Yes | `ParameterDescriptor` with `floating_point_range` is the correct ROS 2 pattern; `configure()` is the right declaration site; implementation should follow ROS 2 Rolling conventions |
| ADR-0013 — progress.md vocabulary | Yes | This entry (Issue Review) |

### Consequences

- `PathPlan` constructor signature changes (margin, max_bend_angle become parameters
  read in `configure()` and passed through) — verify no other callers outside the
  package break.
- `RecordSwath` construction interval becomes a parameter — verify `SurveyPath.cpp:32`
  (`m_swath_record(10)`) is updated to use the declared value.
- The dead `m_swath_interval` member (`SurveyPath.h:71`) must be removed — no other
  update needed since it is never read.

### Recommendations

- Verify that `floating_point_range` enforcement is active for your ROS 2 distro:
  in Rolling/Humble, the node rejects out-of-range values at `declare_parameter` and
  `set_parameter` time, so `ParameterDescriptor` enforcement is real, not just advisory.
  The test for out-of-range rejection should confirm this behavior rather than testing
  only the descriptor metadata.
- The `min_allowable_swath` range (`>= 0`) omits an upper bound — this is fine; omit
  the max or set it to a large sentinel rather than leaving it implicitly unconstrained
  in the descriptor.

### Actions
- [ ] Verify no external callers of `PathPlan(...)` constructor are broken by the signature change.
- [ ] Confirm out-of-range rejection test exercises ROS 2 node-level validation (not just descriptor field inspection).

## Plan Authored
**Status**: complete
**When**: 2026-06-30 17:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-4/plan.md` at `4ae669d`
**Branch**: feature/issue-4 at `4ae669d`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.
