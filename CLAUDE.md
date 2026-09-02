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
