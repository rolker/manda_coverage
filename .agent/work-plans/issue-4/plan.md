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
   `all_zero` check then naturally signals coverage-complete). The threshold is
   scoped to `SwathWidth()` only in Phase 1 — the sibling accessors
   `SwathOuterPts()`/`OuterPoint()`/`AllSwathWidths()` are intentionally left
   un-thresholded (the default threshold of `0.0` is no behavior change today;
   consistent below-threshold handling across the siblings is deferred to a
   later phase).

2. **Declare parameters in `SurveyPath::configure()`** — follow the existing
   `has_parameter` + `declare_parameter` pattern. Add a
   `rcl_interfaces::msg::ParameterDescriptor` with `floating_point_range`
   (min/max/step) and `description`/`additional_constraints` for units.
   Parameters:

   | ROS name | default | range |
   |---|---|---|
   | `swath_overlap` | `0.2` | `from_value=0.0`, `to_value=1.0`, step `0.0` |
   | `max_bend_angle` | `60.0` | `from_value=0.0`, `to_value=90.0`, step `0.0` |
   | `swath_record_interval` | `10.0` | `from_value=0.001`, `to_value=std::numeric_limits<double>::max()` |
   | `min_allowable_swath` | `0.0` | `from_value=0.0`, `to_value=std::numeric_limits<double>::max()` |

   Unbounded-above params use `std::numeric_limits<double>::max()` as a finite
   upper sentinel (requires `#include <limits>`). Do **not** use `to_value=0.0`
   to mean "no upper bound" — that yields a degenerate/zero-width range that
   rejects the default at `configure()` time. Declarations are factored through
   a local `declare_bounded` lambda that builds the descriptor, declares the
   parameter (guarded by `has_parameter`), and returns the read-back value.

3. **Read parameters and apply** — after `declare_parameter`, `get_parameter`
   each value, store in the existing `m_swath_overlap` / `m_max_bend_angle`
   members. Call `m_swath_record.SetInterval()` and
   `m_swath_record.SetMinAllowableSwath()`.

4. **Remove dead member** — delete `m_swath_interval = 10` from `SurveyPath.h`
   and remove the `m_swath_record(10)` from the constructor initializer list
   (let `RecordSwath` use its default of `10`; `configure()` will override it).

5. **Add unit tests** in `test/test_parameters.cpp` using
   `ament_add_ros_isolated_gtest`. The fixture inits/shuts down rclcpp per test
   (no custom `main()`, so no clash with the gtest main ament links). Tests:
   - `SurveyPathParameterTest.DefaultParameters` — create node, build `SurveyPath`,
     call `configure()`, verify each parameter is set to its default via
     `get_parameter`.
   - `SurveyPathParameterTest.OutOfRangeRejection` — after `configure()`, call
     `node->set_parameter()` with values outside each declared range; verify each
     returns `result.successful == false`.
   - `SurveyPathParameterTest.InRangeAccepted` — symmetric check that valid values
     are accepted.
   - `RecordSwathThreshold.{Above,Below,Default}Threshold*` — exercise
     `RecordSwath::SwathWidth()` thresholding directly: a recorded width below
     `min_allowable_swath` returns `0.0` (drives PathPlan's `all_zero`
     coverage-complete check), at/above returns the real width, and the default
     `0.0` threshold never zeroes a positive width.

6. **Update `CMakeLists.txt`** — inside `BUILD_TESTING` block, add
   `find_package(ament_cmake_ros REQUIRED)` (to expose
   `ament_add_ros_isolated_gtest`), declare the `test_parameters` test, and link
   it against the `manda_coverage` library and `rclcpp`.

7. **`package.xml`** — no change needed: `ament_cmake_ros` is already a
   `buildtool_depend`, so no `<test_depend>` add is required.

## Files to Change

| File | Change |
|------|--------|
| `include/manda_coverage/RecordSwath.h` | Add `SetInterval()`, `SetMinAllowableSwath()` public methods |
| `src/RecordSwath.cpp` | Implement setters; threshold `m_min_allowable_swath` in `SwathWidth()` |
| `include/manda_coverage/SurveyPath.h` | Delete `m_swath_interval`; no new members needed |
| `src/SurveyPath.cpp` | Declare 4 params with descriptors in `configure()`; remove `(10)` from constructor initializer |
| `CMakeLists.txt` | Add `find_package(ament_cmake_ros)` + `test_parameters` target in `BUILD_TESTING` block |
| `package.xml` | No change — `ament_cmake_ros` already a `buildtool_depend` |
| `test/test_parameters.cpp` | New: parameter default/range tests + `SwathWidth()` thresholding tests |

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
