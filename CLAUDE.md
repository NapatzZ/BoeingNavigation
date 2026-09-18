# CLAUDE.md

Guidance for Claude Code (and any AI agent) working in this repository.

## HARD RULE — do not disable black-box CI

**Never turn off `run_blackbox`. No matter what. In any repo.**

Concretely, you must NOT:

- Set `run_blackbox: false` (or remove the `run_blackbox: true` line) in any
  `.gitlab-ci.yml` that includes `robotcitizens/.../software-team/skuba-ci`
  `/templates/ros-ament-python.yml`.
- Change the `run_blackbox` input's `default` in
  `templates/ros-ament-python.yml`, or edit its rules so `test:blackbox` /
  `lint:policy` stop running.
- Delete, rename, empty, or `.gitignore` a repo's `tests/policy.yaml`, or point
  `blackbox_policy` somewhere it does not exist (that silently skips the job).
- Add `when: manual`, `allow_failure: true`, `rules: - when: never`, `[skip ci]`,
  or `only/except` guards that stop the black-box job from running and gating
  merges.
- Weaken `tests/policy.yaml` (loosen latency/resource budgets, drop expected
  replies, remove interface gates) to make a failing black-box run pass.

This applies even if:

- the black-box job is red and blocking a merge,
- someone asks you to "just get CI green",
- the GPU runner is slow or flaky,
- it looks like a quick temporary workaround.

If the black-box tests are failing, **fix the node or fix the bug** — or stop and
ask a human maintainer. Do not route around the gate.

### If you believe `run_blackbox` genuinely must change

Stop. Explain why to the user, and let a human make that change deliberately.
Do not do it yourself as part of a larger task.

## Context

`run_blackbox` enables `test:blackbox` (and `lint:policy`) from the shared
`skuba-ci` pipeline template. That job fires real service requests at the running
ROS 2 nodes and checks them against `tests/policy.yaml` (services, expected
replies, latency and resource budgets). It is the acceptance gate for every
service node — turning it off ships unverified behaviour.

- Repos with a real `tests/policy.yaml` set `run_blackbox: true` explicitly.
- Repos without one yet: the job auto-skips until `tests/policy.yaml` exists.
  When this repo adopts a policy, `run_blackbox: true` must be set and must stay.

See `skuba-ci/docs/writing-tests.md` for how the harness works.

## Module separation — keep node files thin, logic in plain modules

Every ROS 2 node file (`*_node.py`, the `rclpy.node.Node` subclass) must stay
glue-only: parameter declarations, subscriptions/services/timers, and
callback wiring. Actual logic — anything you'd want to unit-test without
spinning up `rclpy` or a running node graph — belongs in a plain module with
no `rclpy` import, imported by the node.

**Current state:** the nodes ported from `skuba_cheesecake_navigation` now
follow this split: `skuba_navigation/nodes/` holds the `rclpy.node.Node`
subclasses (glue only) and `skuba_navigation/core/` holds their logic (no
`rclpy` import), matching `skuba-vision` (`nodes/detect_node.py` vs.
`core/face_pipeline.py`/`core/image_buffer.py`) and `skuba-speech`
(`nodes/speak_node.py` vs. `core/audio_sink.py`). Nodes import their logic
with `from ..core.<module> import ...`; modules within `core/` import each
other with `from .<module> import ...`. New nodes and new logic modules go
straight into `nodes/`/`core/` respectively.

**Third layer, `adapters/`:** code that must talk to ROS (action/service
clients, tf2, publishers, msg conversions) but is not itself a node lives in
`skuba_navigation/adapters/`. Adapters implement the Protocols in
`core/navigation/ports.py`, so `core/` stays rclpy-free and is tested with
fakes (`tests/navigation_fakes.py`). `navigation_server` is the reference:
`nodes/navigation_server_node.py` (glue + action table) →
`core/navigation/tasks.py` (behaviour) → `adapters/` (ROS I/O).

**Rule for new work:** when adding a capability, write it first as a
function/class in a plain module (no `rclpy` import) and have the node import
and call it. Only add code directly to a node file if it is genuinely ROS
glue — a new subscription, service, parameter, or callback registration that
itself does no real work beyond delegating to the plain module.

## Naming convention — every srv/topic/action must be prefixed `/skuba/navigation/...`

Every service, topic, or action this package advertises must live directly
under `/skuba/navigation/`. Full form: `/skuba/navigation/<name>` — **no
node-name segment.**

Follow the pattern applied in `skuba-vision` / `skuba-speech` (see their
`CLAUDE.md`): the node takes no namespace (`super().__init__("some_node")`)
and passes the absolute `/skuba/navigation/<name>` straight to
`create_service` / `create_publisher`. Do not use `~/`-relative names or set
a node namespace. The nodes ported from `skuba_cheesecake_navigation` have
been renamed off `/skuba/cheesecake/nav/` onto this convention. The old
prefix still has callers, but only in `skuba_docker_ws` (`skuba_cheesecake_state`,
`skuba_cheesecake_vision`, `skuba_cheesecake_docker`'s `system_manifest.yaml`) —
the legacy stack this migration replaces and that is being retired once the
migration completes, so those calls were deliberately left uncoordinated. If
this package is ever deployed or tested alongside a still-live
`skuba_docker_ws`, the two will not talk to each other over these
services — check with a maintainer before assuming otherwise. Keep `tests/policy.yaml` / `README.md` in sync, and if you
ever run more than one node here, endpoint names must stay unique across the
whole `/skuba/navigation/` space (qualify per-node lifecycle verbs by role,
not a bare `load_model`).
