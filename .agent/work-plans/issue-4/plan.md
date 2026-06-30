# Plan: Expose coverage-density tuning constants as declared parameters

## Issue

https://github.com/rolker/manda_coverage/issues/4

## Context

Four constants controlling coverage density are hardcoded in `SurveyPath` /
`RecordSwath` and cannot be changed without a code rebuild. The issue promotes
them to ROS 2 declared parameters with `ParameterDescriptor` bounds so they
are settable at launch time. Phase 1 only — live/runtime settability is Phase 2.

Current state (verified against source):

- `m_swath_overlap = 0.2` (`SurveyPath.h:73`) — passed to `PathPlan` as `margin`
- `m_max_bend_angle = 60` (`SurveyPath.h:74`) — passed to `PathPlan`
- Swath record interval hardcoded `(10)` in `SurveyPath.cpp:32` constructor initializer
- `m_min_allowable_swath = 0` (`RecordSwath.cpp:23`) — initialized but **never read**;
  needs to be wired into `SwathWidth()` so the PathPlan `all_zero` check respects it
- `m_swath_interval = 10` (`SurveyPath.h:71`) — dead member, never read, delete it

`PathPlan` is only instantiated from `SurveyPath.cpp` — no external callers.

## Approach

1. **Add setters to `RecordSwath`** — `SetInterval(double)` and
   `SetMinAllowableSwath(double)` — so `SurveyPath::configure()` can push
   parameter values after construction. Wire `m_min_allowable_swath` into
   `SwathWidth()`: return `0.0` when width is below the threshold (PathPlan's
   `all_zero` check then naturally signals coverage-complete).

2. **Declare parameters in `SurveyPath::configure()`** — follow the existing
   `has_parameter` + `declare_parameter` pattern. Add a
   `rcl_interfaces::msg::ParameterDescriptor` with `floating_point_range`
   (min/max/step) and `description`/`additional_constraints` for units.
   Parameters:

   | ROS name | default | range |
   |---|---|---|
   | `swath_overlap` | `0.2` | `[0.0, 1.0]`, step `0.0` |
   | `max_bend_angle` | `60.0` | `[0.0, 90.0]`, step `0.0` |
   | `swath_record_interval` | `10.0` | `(0.0, ∞)` — use `from_value=0.001` |
   | `min_allowable_swath` | `0.0` | `[0.0, ∞)` — use `to_value=0.0` to omit upper bound |

3. **Read parameters and apply** — after `declare_parameter`, `get_parameter`
   each value, store in the existing `m_swath_overlap` / `m_max_bend_angle`
   members. Call `m_swath_record.SetInterval()` and
   `m_swath_record.SetMinAllowableSwath()`.

4. **Remove dead member** — delete `m_swath_interval = 10` from `SurveyPath.h`
   and remove the `m_swath_record(10)` from the constructor initializer list
   (let `RecordSwath` use its default of `10`; `configure()` will override it).

5. **Add unit tests** in `test/test_parameters.cpp` using
   `ament_add_ros_isolated_gtest`. Two tests:
   - `TestDefaultParameters` — create node, build `SurveyPath`, call `configure()`,
     verify each parameter is set to its default value via `get_parameter`.
   - `TestOutOfRangeRejection` — after `configure()`, call `node->set_parameter()`
     with values outside each declared range; verify each returns
     `result.successful == false`.

6. **Update `CMakeLists.txt`** — inside `BUILD_TESTING` block, add
   `ament_add_ros_isolated_gtest`, link against the `manda_coverage` library and
   `rclcpp`.

7. **Update `package.xml`** — add `<test_depend>ament_cmake_ros</test_depend>` if
   `ament_add_ros_isolated_gtest` is not already available.

## Files to Change

| File | Change |
|------|--------|
| `include/manda_coverage/RecordSwath.h` | Add `SetInterval()`, `SetMinAllowableSwath()` public methods |
| `src/RecordSwath.cpp` | Implement setters; threshold `m_min_allowable_swath` in `SwathWidth()` |
| `include/manda_coverage/SurveyPath.h` | Delete `m_swath_interval`; no new members needed |
| `src/SurveyPath.cpp` | Declare 4 params with descriptors in `configure()`; remove `(10)` from constructor initializer |
| `CMakeLists.txt` | Add test target in `BUILD_TESTING` block |
| `package.xml` | Add `ament_cmake_ros` test depend if missing |
| `test/test_parameters.cpp` | New: parameter default + out-of-range tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Promotes hardcoded knobs to operator-visible parameters with descriptor bounds |
| A change includes its consequences | Tests cover defaults and range enforcement; dead member retired |
| Only what's needed | Phase 1 scope only — no runtime settability, no extra abstraction |
| Test what breaks | Tests exercise the ROS 2 parameter declaration behavior, not just metadata |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Working on `feature/issue-4` branch |
| ADR-0008 — Follow ROS 2 conventions | Yes | `ParameterDescriptor` with `floating_point_range` in `configure()` is the correct ROS 2 pattern |
| ADR-0013 — progress.md vocabulary | Yes | Plan Authored entry follows |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `RecordSwath` — add setters | `SurveyPath::configure()` calls them | Yes |
| `SwathWidth()` thresholds | Coverage-complete behavior changes when `min_allowable_swath > 0` | Yes — this is the intent |
| `PathPlan` constructor callers | None outside package | Verified — no action |

## Open Questions

- None — scope is fully defined by the issue; all file paths verified against source.

## Estimated Scope

Single PR.
