# Plan: Make coverage-planner parameters live-settable at runtime

## Issue

https://github.com/rolker/manda_coverage/issues/5

## Context

Issue #4 (PR #7, now merged) declared all coverage-density parameters with
`ParameterDescriptor` floating-point-range constraints. Those descriptors enable
ROS 2 to reject out-of-range values at the node layer — but there are no
parameter-set callbacks, so accepted `ros2 param set` calls never
propagate into the cached member variables or live `RecordSwath` state.
This PR wires a validate/apply callback pair so operators can tune parameters
between survey lines without restarting the node.

Key design decision (operator-chosen): **split validate from apply** using the
idiomatic Jazzy/Rolling pattern — an `add_on_set_parameters_callback` that only
validates the proposed values (no mutation), and an
`add_post_set_parameters_callback` that applies committed values. This is
atomic-safe for multi-parameter sets: a later rejected value cannot leave
earlier members half-applied.

Key design decision (from review-issue, open-action): **setter over re-read per
cycle** for `RecordSwath.SetInterval()`. `RecordSwath` already has public
`SetInterval()` and `SetMinAllowableSwath()` setters; the post-set apply
callback calls them directly. `PathPlan` is constructed fresh in `CreateNewPath()` from cached
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

3. **Split validate (on-set) from apply (post-set) in `configure()`** — After
   all parameter declarations, register two callbacks (the idiomatic
   Jazzy/Rolling pattern, which is atomic-safe for multi-parameter sets):

   - **`add_on_set_parameters_callback` = VALIDATE only.** Iterates the proposed
     `const std::vector<rclcpp::Parameter>&` and returns
     `rcl_interfaces::msg::SetParametersResult` — `successful = true` on accept,
     or `successful = false` with a populated `reason` string on a type
     mismatch. It does **not** write any cached member or call a `RecordSwath`
     setter. Descriptor floating-point-range rejections are produced by rclcpp
     *before* this callback fires, so out-of-range sets are already rejected
     upstream; the test's `result.successful == false` for out-of-range comes
     from ROS, not this callback. The `reason` string is read by the Phase 3
     marine_control bridge for operator-facing diagnostics.
   - **`add_post_set_parameters_callback` = APPLY.** Runs *after* the set is
     committed. Iterates the committed `const std::vector<rclcpp::Parameter>&`
     and, for each recognised name, updates the cached member
     (`m_swath_overlap`, `m_max_bend_angle`, the three distance members) and/or
     calls the live `RecordSwath` setter (`SetInterval`,
     `SetMinAllowableSwath`). This is where live state actually changes — and,
     because it runs only on a committed set, a multi-parameter set whose later
     value is rejected never leaves earlier members half-applied.

4. **Store both callback handles in `SurveyPath.h`** — Add
   `rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
   on_set_param_callback_handle_` and
   `rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
   post_set_param_callback_handle_` as private members. Reset **both** in
   `cleanup()` so the callbacks are unregistered during lifecycle teardown.

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

7. **Extend `test_parameters.cpp`** — Add two tests that go beyond the existing
   `InRangeAccepted`/`OutOfRangeRejection` (which only assert
   `SetParametersResult`) by asserting state via the new getters:
   `InRangeSetUpdatesLiveState` sets each live parameter in-range and asserts
   the cached member / live `RecordSwath` value changed (proving the post-set
   apply callback fired), and `RejectedSetLeavesLiveStateUnchanged` confirms an
   out-of-range set is rejected *and* leaves the getter value untouched. No new
   test file — extend the existing one per the review-issue consequence note.

## Files to Change

| File | Change |
|------|--------|
| `src/SurveyPath.cpp` | Mark `soundings_topic`/`display_topic` read-only; use `declare_bounded` for distance params; add on-set (validate) + post-set (apply) callbacks; reset both handles in `cleanup()` |
| `include/manda_coverage/SurveyPath.h` | Add `on_set_param_callback_handle_` and `post_set_param_callback_handle_` members; add four const getter methods |
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
| `waypoint_distance_threshold` gets a `[0, ∞)` descriptor | Existing nodes that set this param to a negative value will be rejected — negatives are nonsensical anyway | Yes (descriptor added; implicit change) |
| `lead_in_distance` gets a `[0, ∞)` descriptor | Same benign effect — negative lead-in values rejected | Yes (descriptor added; implicit change) |
| `lead_out_distance` gets a `[0, ∞)` descriptor | Same benign effect — negative lead-out values rejected | Yes (descriptor added; implicit change) |
| `GetMinAllowableSwath()` added to `RecordSwath` | None — pure addition | Yes |
| Validate/apply split (on-set + post-set callbacks) | Both handles must be reset in `cleanup()` so they unregister on lifecycle teardown | Yes (step 4) |

## Open Questions

- None — review-issue cleared all scope questions; dependency (#4) is merged.

## Estimated Scope

Single PR.
