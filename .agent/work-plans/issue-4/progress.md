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

## Plan Review
**Status**: complete
**When**: 2026-06-30 15:23 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Name matches the Plan Authored entry, but this is a fresh-context Opus sub-agent reviewing a Sonnet-authored plan — genuinely independent, not an in-context self-review, so no self-review annotation. -->

**Plan**: `.agent/work-plans/issue-4/plan.md` at `4ae669d`
**PR**: PR-less (--issue mode; gh unauthenticated — issue context read from the Issue Review entry above)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Range descriptors for unbounded-above params are encoded wrong: `swath_record_interval` `from_value=0.001` with default `to_value=0.0` rejects the default 10.0 at declare time (configure() throws); `min_allowable_swath` `to_value=0.0` yields range `[0,0]` so the threshold can never be raised. Use finite upper sentinels (e.g. `numeric_limits<double>::max()`) per the Issue Review's "large sentinel" recommendation — there is no existing `ParameterDescriptor` pattern in the repo to copy. — `plan.md:43-44`
- [ ] (suggestion) "Test what breaks": tests cover only ROS-level declaration/range rejection; the new `SwathWidth()` thresholding logic (feeds PathPlan `all_zero` at `PathPlan.cpp:97-103` and zeroes the offset at `PathPlan.cpp:86`) is untested. Add a `RecordSwath::SwathWidth()` unit test or document the gap. — `plan.md:55-62`
- [ ] (suggestion) BUILD_TESTING block needs `find_package(ament_cmake_ros REQUIRED)` to expose `ament_add_ros_isolated_gtest`; current block only runs lint. `ament_cmake_ros` is already a buildtool_depend, so step 7's test_depend add is a no-op. — `plan.md:63-68`

### Notes
- Structurally sound: file targeting, dead-member (`m_swath_interval`) and never-read (`m_min_allowable_swath`) claims, and the "PathPlan ctor signature unchanged → no external-caller breakage" consequence all verified against source. The must-fix is a concrete correctness defect in the parameter-range encoding, not a structural re-plan.

## Implementation
**Status**: complete
**When**: 2026-06-30 16:05 +00:00
**By**: Claude Opus

**Branch**: feature/issue-4 at `b9ec729`
**Commits**:
- `c231faf` — RecordSwath setters (`SetInterval`, `SetMinAllowableSwath`) + `SwathWidth()` thresholding
- `444f9ba` — SurveyPath parameter declarations + dead-member removal
- `9f6886a` — tests + CMake `BUILD_TESTING` wiring
- `b9ec729` — plan sync

### What was implemented
Implemented the plan as written with all three Plan Review corrections applied:

1. **Range-descriptor encoding (MUST-FIX, applied).** Parameters declared in
   `SurveyPath::configure()` via a local `declare_bounded` lambda using
   `FloatingPointRange` descriptors:
   - `swath_overlap` `[0.0, 1.0]` default `0.2`
   - `max_bend_angle` `[0.0, 90.0]` default `60.0`
   - `swath_record_interval` `[0.001, std::numeric_limits<double>::max()]` default `10.0`
   - `min_allowable_swath` `[0.0, std::numeric_limits<double>::max()]` default `0.0`
   Finite `max()` sentinels used for unbounded-above ranges (no `to_value=0.0`);
   `#include <limits>` added.

2. **SwathWidth() thresholding test (SUGGESTION, applied).** `RecordSwathThreshold`
   suite covers below-threshold → `0.0`, at/above → real width, and default `0.0`
   threshold never zeroing a positive width. (A test-helper bug — first `AddRecord`
   at `(0,0,heading=0)` collided with the zero-initialised `m_previous_record`
   duplicate guard and was silently dropped — was found and fixed by using
   heading `45.0`.)

3. **CMake test dependency (SUGGESTION, applied).** Added
   `find_package(ament_cmake_ros REQUIRED)` to the `BUILD_TESTING` block to expose
   `ament_add_ros_isolated_gtest`. Plan step 7's `package.xml` `<test_depend>` add
   was correctly skipped as a **no-op** — `ament_cmake_ros` is already a
   `buildtool_depend`.

Dead `m_swath_interval` member removed from `SurveyPath.h`; redundant
`m_swath_record(10)` constructor initialiser dropped (defaults to `10`,
overridden by `configure()`).

### Build & test
- `./core_ws/build.sh manda_coverage` — clean (built via `--packages-up-to` to
  pick up in-workspace deps `marine_nav_interfaces`/`marine_nav_utilities` on
  first build; only pre-existing `-Wunused-parameter` warnings in
  `action_server.cpp`, none from this change).
- gtest `test_parameters`: **6/6 pass** (3 parameter + 3 thresholding).
- The package's `ament_lint` suite has a large pre-existing baseline of
  uncrustify/cpplint/copyright failures across the legacy MOOS-derived sources
  (header-guard style, comment spacing, missing copyright, etc.) — out of scope
  for this issue. The new `test/test_parameters.cpp` is lint-clean (uncrustify,
  cpplint, copyright all pass) and the one >100-char line introduced in
  `SurveyPath.cpp` was wrapped so no new cpplint line-length failure is added.

### Next step
Ready for review-code. No push/PR performed (host publishes after local review).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 16:10 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-4 at `df6b2a9`
**Mode**: pre-push
**Depth**: Standard (reason: multi-file C++ source touching ROS 2 parameter API + new test target; single package)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; clean, plan-faithful Phase-1 increment, default behaviour unchanged

### Findings
- [x] (suggestion) `min_allowable_swath` threshold applied in `SwathWidth()` but not `SwathOuterPts`/`OuterPoint`/`AllSwathWidths` — partial below-threshold coverage keeps `all_zero` false and zero-offsets sub-threshold points; document or apply consistently + add PathPlan integration test (cross-pass confirmed: Lens A + Lens B) — `src/RecordSwath.cpp:217` / `src/PathPlan.cpp:85` (documented per operator decision: scoping note added in code + plan.md; consistent threading + integration test deferred to a later phase)
- [x] (suggestion) Boundary `width == min_allowable_swath` (strict `<` keeps it as valid) is untested — add a boundary test — `src/RecordSwath.cpp:217`
- [ ] (suggestion) Redundant trailing blank line at end of `configure()` (only new lint finding on a touched line) — `src/SurveyPath.cpp:131`

### Notes
- Plan adherence strong: all planned files changed, no scope creep; finite `numeric_limits<double>::max()` range sentinels, dead `m_swath_interval` retired, `find_package(ament_cmake_ros)` added, `package.xml` correctly unchanged.
- Range enforcement verified active at node `set_parameter` level (not just descriptor metadata). Build/CMake/test wiring clean; new `test/test_parameters.cpp` is lint-clean. Legacy MOOS lint baseline is pre-existing and not attributed to this PR.
- Tests reported 6/6 passing in the Implementation entry; not independently re-run this session.

### Operator decision (publish checkpoint, 2026-06-30)
Address all three suggestions before publishing, per these per-finding decisions:
- **(1) Document, do NOT apply.** Keep `min_allowable_swath` thresholding scoped to
  `SwathWidth()` only (it drives PathPlan's `all_zero` coverage-complete check).
  Do **not** thread it through `SwathOuterPts`/`OuterPoint`/`AllSwathWidths`. Add a
  short Phase-1 scoping note in the code (comment near the `SwathWidth()` threshold
  and/or the param declaration) explaining that consistent below-threshold handling
  across the sibling accessors is deferred (default threshold `0.0` ⇒ no behavior
  change today; full coverage-complete semantics with a non-zero threshold is a
  later phase). Reflect the same note in `plan.md` if it touches that behavior.
- **(2) Apply.** Add the `width == min_allowable_swath` boundary test (strict `<`
  ⇒ value at exactly the threshold stays valid / returns the real width).
- **(3) Apply.** Remove the redundant trailing blank line at end of `configure()`
  (`src/SurveyPath.cpp:131`).
Re-run the package build + tests; then the host re-dispatches review-code.
