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

**Current state:** `skuba_navigation/` is still an empty stub
(`__init__.py` only). Apply this split from the first real node you add —
create the logic as a plain module under `skuba_navigation/` before writing
the node class, don't retrofit the split later. See `skuba-vision` (e.g.
`detect_node.py` vs. `face_pipeline.py`/`image_buffer.py`) or `skuba-speech`
(`speak_node.py` vs. `_audio_sink.py`) for the pattern already in use
elsewhere in this workspace.

**Rule for new work:** when adding a capability, write it first as a
function/class in a plain module (no `rclpy` import) and have the node import
and call it. Only add code directly to a node file if it is genuinely ROS
glue — a new subscription, service, parameter, or callback registration that
itself does no real work beyond delegating to the plain module.

## Naming convention — every srv/topic/action must be prefixed `/skuba/navigation/...`

Every service, topic, or action this package advertises must live under the
`/skuba/navigation/` namespace. Full form:
`/skuba/navigation/<node_name>/<name>`.

Follow the pattern already applied in `skuba-vision` and `skuba-speech`
(see their `CLAUDE.md`): either set the node's `namespace="skuba/navigation"`
in `super().__init__(...)` and use `~/`-relative names (the `speak_node`
style), or hardcode the full absolute path per endpoint (the `detect_node`
style) — pick one and apply it consistently across this package's nodes.
This repo has no nodes yet; apply the prefix from the first service/topic/
action you add, and keep any `tests/policy.yaml` and `README.md` in sync
with whatever names you pick.
