#!/usr/bin/env bash
# Hand run of manda_coverage's action server, closed loop, for issue #11.
#
# usage: hand_run.sh <setup.bash> <out_dir> [ros_domain_id]
#
# Sources <setup.bash> (a built workspace's install/setup.bash, with
# manda_coverage and nav2_lifecycle_manager in it), checks that the ROS domain
# (third argument, else $ROS_DOMAIN_ID; it must be set and not 0) has no nodes,
# starts manda_coverage_action_server, brings it up with nav2_lifecycle_manager
# (autostart, which also makes the bond), records its lifecycle state and
# parameters, then runs coverage_probe.py (next to this script) with the survey
# polygon in both vertex orders, one goal after the other on the same node, and
# stops everything it started. Everything goes to <out_dir>:
#   env.txt                 distro, domain, package prefixes, date
#   lifecycle_before.txt    `ros2 lifecycle get` after bring-up
#   params.txt              `ros2 param list` of the server
#   lines_fwd.txt / lines_rev.txt     the returned lines, for diffing
#   summary_fwd.txt / summary_rev.txt the probe's checks and verdict
#   probe_fwd.log / probe_rev.log     the probe's output
#   lifecycle_after.txt     state after both goals (the node must still be up)
#   server.log / manager.log
#
# What the checks are: a sanity check for a flat 40 m swath (success within 10
# lines, vertices inside the polygon, lines parallel to the first edge and
# 32 to 40 m apart, the strips covering most of the polygon). They do not check
# the planner's adaptive behaviour, and the run does not show the behaviour
# tree or mission_manager calling the action, Nav2 following the lines, real
# sonar geometry or TF, or a real survey.
#
# For the owner, on Jazzy (deadpool): build the jazzy branch before the
# backport and run this into one folder, then build it after the backport and
# run this into another, then
#     diff before/lines_fwd.txt after/lines_fwd.txt
#     diff before/lines_rev.txt after/lines_rev.txt
# The backport includes the RecordSwath guard, a decided behaviour change.
# Before it, the turn check read an empty list for every record from the
# second up to the first MinInterval() of each line (undefined behaviour; the
# heading read is indeterminate, uninitialised padding in a list header: in
# practice usually about 0, but it can be NaN, which gives no override, the
# same as guarded, or a huge value, on which angle180 never returns and the
# node hangs) and, with a heading of about 0, when the boat's heading and
# the output side met the turn condition, dropped the line's first record. In
# this polygon the condition is met only in the "fwd" order (counter-clockwise:
# first line on Port at heading 60, the next on Stbd at heading -120, and so
# on); in "rev", the reversed (clockwise) order, it is never met. So:
#   - "rev" is expected identical before and after (within 1e-6 m): a
#     difference there is not the guard's and is a bug to report.
#   - "fwd" is the order in which the guard is expected to make a difference
#     after the backport, near the start of each recorded line. With this flat
#     40 m swath the difference may be nil: on Rolling, this probe gave the
#     same lines, to 1e-6 m in both orders, from the guarded node at -O0 and
#     from an unguarded node at -O2; in a scratch harness the change showed only
#     with a varying swath (see progress.md, ## Implementation). Either result
#     is worth sending back.
# Send back: both folders whole (at least env.txt, lifecycle_*.txt,
# summary_*.txt, lines_*.txt and server.log), the output of the two diffs, and
# the jazzy commit each folder was built from.
if [ $# -lt 2 ]; then
  echo "usage: $0 <setup.bash> <out_dir> [ros_domain_id]" >&2
  exit 2
fi
SETUP="$1"
OUT="$2"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SETUP"
# After sourcing: ROS setup files do not run under set -u.
set -u
export ROS_DOMAIN_ID="${3:-${ROS_DOMAIN_ID:-0}}"
if [ "$ROS_DOMAIN_ID" = "0" ]; then
  echo "set a ROS_DOMAIN_ID of your own (third argument or environment)" >&2
  exit 2
fi
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

nodes="$(ros2 node list --no-daemon 2>/dev/null)"
if [ -n "$nodes" ]; then
  echo "ROS_DOMAIN_ID $ROS_DOMAIN_ID is not empty:" >&2
  echo "$nodes" >&2
  exit 3
fi

{
  echo "ROS_DISTRO=${ROS_DISTRO:-?} ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=${RMW_IMPLEMENTATION:-default}"
  echo "manda_coverage: $(ros2 pkg prefix manda_coverage)"
  echo "nav2_lifecycle_manager: $(ros2 pkg prefix nav2_lifecycle_manager)"
  echo "date: $(date '+%Y-%m-%d %H:%M %z')"
} > "$OUT/env.txt"

SERVER="$(ros2 pkg prefix manda_coverage)/lib/manda_coverage/manda_coverage_action_server"
MANAGER="$(ros2 pkg prefix nav2_lifecycle_manager)/lib/nav2_lifecycle_manager/lifecycle_manager"
SERVER_PID=""
MANAGER_PID=""

cleanup() {
  for pid in $MANAGER_PID $SERVER_PID; do
    kill -INT "$pid" 2>/dev/null
  done
  for _ in $(seq 1 50); do
    alive=""
    for pid in $MANAGER_PID $SERVER_PID; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ -z "$alive" ] && return
    sleep 0.2
  done
  for pid in $MANAGER_PID $SERVER_PID; do
    kill -KILL "$pid" 2>/dev/null
  done
}
trap cleanup EXIT

"$SERVER" > "$OUT/server.log" 2>&1 &
SERVER_PID=$!
"$MANAGER" --ros-args -r __node:=lifecycle_manager_coverage \
  -p "node_names:=['manda_coverage_action_server']" -p autostart:=true \
  > "$OUT/manager.log" 2>&1 &
MANAGER_PID=$!

state=""
for _ in $(seq 1 60); do
  state="$(ros2 lifecycle get --no-daemon /manda_coverage_action_server 2>/dev/null)"
  case "$state" in active*) break ;; esac
  sleep 1
done
echo "$state" > "$OUT/lifecycle_before.txt"
echo "lifecycle state after bring-up: $state"
case "$state" in
  active*) ;;
  *) echo "server did not become active" >&2; exit 4 ;;
esac
ros2 param list --no-daemon /manda_coverage_action_server > "$OUT/params.txt" 2>&1

rc=0
for order in fwd rev; do
  echo "== probe, order $order"
  python3 "$HERE/coverage_probe.py" "$order" "$OUT" 300 > "$OUT/probe_$order.log" 2>&1 || rc=1
  tail -n 12 "$OUT/probe_$order.log"
done

if kill -0 "$SERVER_PID" 2>/dev/null; then
  ros2 lifecycle get --no-daemon /manda_coverage_action_server > "$OUT/lifecycle_after.txt" 2>&1
else
  echo "server process exited" > "$OUT/lifecycle_after.txt"
  rc=1
fi
echo "lifecycle state after both goals: $(cat "$OUT/lifecycle_after.txt")"
echo "results in $OUT"
exit $rc
