# Plan: Adopt marine_control ControlServer for bridge-side operator control

## Issue

https://github.com/rolker/manda_coverage/issues/6

## Context

`manda_coverage` is a `nav2_util::LifecycleNode`. Its seven tuning parameters
(`swath_overlap`, `max_bend_angle`, `swath_record_interval`,
`min_allowable_swath`, `waypoint_distance_threshold`, `lead_in_distance`,
`lead_out_distance`) are already declared with typed, range-bounded descriptors in
`SurveyPath::configure()` (issue #4 done) and are live-settable via a post-set
callback (issue #5 done). `udp_bridge` carries only pub/sub — ROS 2 parameters are
invisible topside — so control must go through the ADR-0003 topic contract:
`~/control/state` (device→operator) and `~/control/change` (operator→device).

`marine_control::ControlServer` implements that contract. Constructed with the
node, it publishes a RELIABLE+VOLATILE `ControlSet` heartbeat and applies
inbound `ControlValue` via `set_parameter`, delegating clamping to the
node's existing on-set validation. The server is active as soon as constructed,
so lifecycle gating requires constructing it in `on_activate()` and resetting it
in `on_deactivate()`.

## Approach

1. **Add build dependency on `marine_control`** — `package.xml` + `CMakeLists.txt`; link against the action-server executable.

2. **Add `ControlServer` member to `MandaCoverageActionServer`** — declare as `std::unique_ptr<marine_control::ControlServer> control_server_` in the header; include `marine_control/control_server.hpp`.

3. **Construct in `on_activate()`, bind all seven parameters** — after `survey_path_->activate()`:
   ```cpp
   marine_control::ControlServerOptions opts;
   opts.device_name = "Manda Coverage";
   control_server_ = std::make_unique<marine_control::ControlServer>(this, opts);
   control_server_->bind_parameter("swath_overlap",              "fraction", "Coverage");
   control_server_->bind_parameter("max_bend_angle",             "deg",      "Coverage");
   control_server_->bind_parameter("swath_record_interval",      "m",        "Coverage");
   control_server_->bind_parameter("min_allowable_swath",        "m",        "Coverage");
   control_server_->bind_parameter("waypoint_distance_threshold","m",        "Coverage");
   control_server_->bind_parameter("lead_in_distance",           "m",        "Coverage");
   control_server_->bind_parameter("lead_out_distance",          "m",        "Coverage");
   ```

   **Deadlock analysis (review-issue Action #2, carried forward).** An inbound
   `ControlValue` triggers `ControlServer::on_change` → `node->set_parameter`,
   which synchronously fires the post-set apply callback (`m_param_mutex`, from
   #5). The apply runs **inline on `on_change`'s own thread** — rclcpp invokes
   post-set callbacks synchronously inside `set_parameter`, not as a separate
   callback-group dispatch — so `m_param_mutex` is acquired and released within
   that one call. This chain is deadlock-free: the apply callback only copies
   validated values into cached members / `RecordSwath` and never calls
   `set_parameter` or re-enters the ControlServer, so `m_param_mutex` is the only
   lock held across it and there is no lock-order inversion. With the
   `SingleThreadedExecutor` (below) there is moreover no concurrency between the
   lifecycle bind/reset transitions and the server's own callbacks. A short
   version of this note lives in `on_activate()` next to the construction.

4. **Reset in every teardown path** — `control_server_.reset()` placed before the
   existing `survey_path_` teardown in **`on_deactivate()`, `on_cleanup()`, and
   `on_shutdown()`** (review-plan suggestion 3). The server is active the moment
   it is constructed, and `control_server.hpp:38` warns against destroying it from
   the node destructor while the node may still be spinning. A direct
   active→shutdown skips `on_deactivate`, so resetting only there would leak the
   teardown to the destructor; resetting in all three paths closes that gap.
   `reset()` is idempotent on a null `unique_ptr`, so each path is safe when the
   server is already gone (e.g. cleanup after deactivate, or a node that was
   configured but never activated).

5. **SingleThreadedExecutor** (pre-push review round 1, must-fix) — `src/main.cpp`
   spins the node on a `rclcpp::executors::SingleThreadedExecutor` rather than a
   `MultiThreadedExecutor`. The ControlServer is constructed/bound in `on_activate`
   and reset in the teardown paths while the node spins; its own
   mutually-exclusive callback group could otherwise run on a different thread than
   the lifecycle transitions, racing the unsynchronized `bindings_` map during bind
   and racing timer/sub teardown against an in-flight heartbeat during reset — UB
   per `control_server.hpp`'s "bind before spinning / destroy only when not
   spinning" contract. A single executor thread serializes every callback, so both
   races are structurally impossible. `m_param_mutex` (#5) is **retained**
   defensively to document and guard the param-apply-vs-planning invariant should
   the executor ever change back; the now-stale "MultiThreadedExecutor" rationale
   comments in `SurveyPath.{h,cpp}` are updated to say so.

6. **Lifecycle test** (review-issue Action #1 / review-plan suggestion 1) —
   `test/test_control_server_lifecycle.cpp`, wired via `ament_add_ros_isolated_gtest`.
   It drives `MandaCoverageActionServer` through `configure()`/`activate()` on a
   `SingleThreadedExecutor` (matching production), asserts a `ControlSet` heartbeat
   arrives on `~/control/state` with the seven bound knobs (`group == "Coverage"`),
   then `deactivate()`s and asserts the heartbeat stops (teardown verified), then
   **re-activates** and asserts the heartbeat resumes with the seven re-bound knobs
   (the deactivate→activate re-bind path, pre-push review suggestion). The poll
   loop uses `spin_some` + a steady-clock deadline (no fixed sleeps), and the
   pre-deactivate baseline is captured only after a bounded drain loop so an
   in-flight RELIABLE heartbeat cannot read as a spurious "heartbeat continued".
   This covers the `on_activate`/`on_deactivate` adoption paths that the
   `rclcpp::Node`-based `test_parameters.cpp` cannot reach.

## Files to Change

| File | Change |
|------|--------|
| `package.xml` | Add `<depend>marine_control</depend>`; add `<test_depend>marine_control_interfaces</test_depend>` for the lifecycle test |
| `CMakeLists.txt` | Add `find_package(marine_control REQUIRED)`; add `marine_control::marine_control` to `target_link_libraries` for the action-server executable; in `BUILD_TESTING`, `find_package(marine_control_interfaces)` and register `test_control_server_lifecycle` (compiling `src/action_server.cpp` into it) |
| `include/manda_coverage/action_server.h` | Include `marine_control/control_server.hpp`; add `std::unique_ptr<marine_control::ControlServer> control_server_` private member |
| `src/action_server.cpp` | Construct + bind in `on_activate()` (with the deadlock-analysis comment); `control_server_.reset()` in `on_deactivate()`, `on_cleanup()`, and `on_shutdown()` |
| `src/main.cpp` | Spin on `SingleThreadedExecutor` (not `MultiThreadedExecutor`) so ControlServer callbacks never race the lifecycle bind/reset (pre-push review must-fix) |
| `include/manda_coverage/SurveyPath.h`, `src/SurveyPath.cpp` | Update the `m_param_mutex` rationale comments: the node is now single-threaded, so the mutex is retained defensively (not for live MTExecutor concurrency) |
| `test/test_control_server_lifecycle.cpp` | New gtest exercising configure→activate (heartbeat with seven bound knobs), deactivate (heartbeat stops), and re-activate (heartbeat resumes); bounded drain before the deactivate baseline |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Bridge wiring (`udp_bridge` config) is explicitly out of scope per the issue; no stale docs left. The adoption path is now covered by `test_control_server_lifecycle.cpp` (review-issue Action #1). |
| Only what's needed | Seven `bind_parameter()` calls plus a focused lifecycle test — no new abstractions. The `ControlServer`'s internals are tested in `marine_control`; this test only asserts the adoption (bind set, group, teardown), not duplicated server behaviour. |
| Lifecycle gating | Construct in `on_activate()`, reset in `on_deactivate()`/`on_cleanup()`/`on_shutdown()` — covering the direct active→shutdown path the header warns about. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 (bridgeable device control) | Yes | Using `ControlServer` is the ADR-0003 D4/D5/D6 implementation: RELIABLE+VOLATILE QoS, no TRANSIENT_LOCAL, heartbeat republish, fire-and-forget `set_parameter`. |
| ADR-0008 (ROS 2 conventions) | Yes | Parameters already declared with descriptors; QoS and topic names follow ROS 2 conventions. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `manda_coverage` now publishes `~/control/state` | `udp_bridge` config in `unh_echoboats_project11` must bridge these topics | No — tracked on issue #358 (explicit out of scope) |
| `ControlServer` constructed on activate | The node must be activated before topside can see or change params | By design; documented in issue body |

## Open Questions

- None. `bind_parameter()` is called after parameters are declared (by
  `on_configure` → `survey_path_->configure()`); the lifecycle gating pattern is
  specified in the `ControlServer` header; and the three review-plan suggestions
  (reset in all teardown paths, deadlock analysis, lifecycle test) are all folded
  into the approach above and implemented.

## Estimated Scope

Single PR.
