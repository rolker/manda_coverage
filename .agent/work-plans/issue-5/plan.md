# Plan: Make coverage-planner parameters live-settable at runtime

## Issue

https://github.com/rolker/manda_coverage/issues/5

## Context

Issue #4 (PR #7, now merged) declared all coverage-density parameters with
`ParameterDescriptor` floating-point-range constraints. Those descriptors enable
ROS 2 to reject out-of-range values at the node layer — but there is no
`add_on_set_parameters_callback`, so accepted `ros2 param set` calls never
propagate into the cached member variables or live `RecordSwath` state.
This PR wires the callback so operators can tune parameters between survey lines
without restarting the node.

Key design decision (from review-issue, open-action): **setter over re-read per
cycle** for `RecordSwath.SetInterval()`. `RecordSwath` already has public
`SetInterval()` and `SetMinAllowableSwath()` setters; the callback calls them
directly. `PathPlan` is constructed fresh in `CreateNewPath()` from cached
members each time, so updating `m_swath_overlap` / `m_max_bend_angle` is
sufficient — no `PathPlan` setter is needed.

`soundings_topic` and `display_topic` are marked `read_only` in their
descriptors; ROS 2 rejects `set_parameter` calls for them before any callback
is invoked.

## Approach

1. **Mark `soundings_topic` and `display_topic` as `read_only`** — In
   `SurveyPath::configure()`, replace bare `declare_parameter()` calls with
   versions that pass a `ParameterDescriptor` with `read_only = true` and a
   description string. This makes restart-only semantics explicit and prevents
   the on-set callback from needing to handle them.

2. **Add bounded descriptors for the three distance parameters** — Declare
   `waypoint_distance_threshold_`, `lead_in_distance_`, and `lead_out_distance_`
   using `declare_bounded()` (the existing lambda in `configure()`) with
   `from_value = 0.0, to_value = std::numeric_limits<double>::max()`. This
   gives ROS 2 automatic range enforcement and uniform declaration style.

3. **Register `add_on_set_parameters_callback` in `configure()`** — After all
   parameter declarations, call
   `parameter_interface->add_on_set_parameters_callback(...)` with a lambda
   that iterates `const std::vector<rclcpp::Parameter>&` and, for each
   recognised name, updates the cached member and/or live object. Return
   `rcl_interfaces::msg::SetParametersResult` with `successful = true` and a
   populated `reason` string on any failure (the Phase 3 marine_control bridge
   reads `reason` for operator-facing diagnostics). The descriptor-range check
   is enforced by ROS 2 before the callback fires, so the callback only needs
   to handle the live-state update — but still populates `reason` on any
   unexpected type mismatch.

4. **Store the callback handle in `SurveyPath.h`** — Add
   `rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
   param_callback_handle_` as a private member. Reset it in `cleanup()` so the
   callback is unregistered during lifecycle teardown.

5. **Add `GetMinAllowableSwath()` getter to `RecordSwath.h`** — The existing
   `SetMinAllowableSwath()` has no paired getter; add
   `double GetMinAllowableSwath() const { return m_min_allowable_swath; }` so
   the new test can verify the live state update without relying on internal
   access.

6. **Add test-accessor methods to `SurveyPath.h`** — Add four `const` getters
   so tests can verify cached members without friend-class access:
   `double swath_overlap() const`, `double max_bend_angle() const`,
   `double swath_record_interval() const` (delegates to
   `m_swath_record.IntervalDist()`), and `double min_allowable_swath() const`
   (delegates to `m_swath_record.GetMinAllowableSwath()`).

7. **Extend `test_parameters.cpp`** — Add a new fixture/test group
   `SurveyPathLiveParamTest` that sets each live parameter in-range, confirms
   `result.successful == true`, and asserts the cached member/live-object
   value changed via the new getters. Add companion out-of-range rejection
   tests that confirm `result.successful == false` and the state is unchanged.
   Also test each distance param in-range/out-of-range. No new test file —
   extend the existing one per the review-issue consequence note.

## Files to Change

| File | Change |
|------|--------|
| `src/SurveyPath.cpp` | Mark `soundings_topic`/`display_topic` read-only; use `declare_bounded` for distance params; add `add_on_set_parameters_callback` with member-update logic |
| `include/manda_coverage/SurveyPath.h` | Add `param_callback_handle_` member; add four const getter methods |
| `include/manda_coverage/RecordSwath.h` | Add `GetMinAllowableSwath()` getter |
| `test/test_parameters.cpp` | Extend with live-update and rejection tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Live params + operator-readable `reason` strings directly serve this; `read_only` on topic params makes restart semantics explicit |
| A change includes its consequences | Tests verify both cached-member and live-object state; `read_only` declarations are a public API change documented in PR description |
| Test what breaks | Tests target the failure mode (stale state after `ros2 param set`) that prompted this issue; range rejection tests guard Phase 3 safety gating |
| Only what's needed | Getters added are the minimum to make the live-update contract testable; no speculative additions |
| Improve incrementally | Single PR, Phase 2 of three-phase operator-control rollout |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (ROS 2 conventions) | Yes | `add_on_set_parameters_callback` returning `rcl_interfaces::msg::SetParametersResult` is the standard pattern; registration done in `on_configure` equivalent (`configure()`) after all declarations, matching lifecycle best practice |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `soundings_topic`/`display_topic` declared `read_only` | Any operator scripts that attempt `ros2 param set` on these will start failing — call out in PR description | No (PR description) |
| `waypoint_distance_threshold_` gets a descriptor | Existing nodes that set this param outside `[0, ∞)` will be rejected — only zero/negative values, which are nonsensical anyway | Yes (descriptor added; implicit change) |
| `GetMinAllowableSwath()` added to `RecordSwath` | None — pure addition | Yes |

## Open Questions

- None — review-issue cleared all scope questions; dependency (#4) is merged.

## Estimated Scope

Single PR.
