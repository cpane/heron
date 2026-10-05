# Contributing to Heron

Heron is a robot platform on a Raspberry Pi, brought up one subsystem at a
time. This guide covers how branches, pull requests and releases work, and
what a change needs before it is merged. How to build, deploy and run things
is in the [README](README.md).

## Branching model

Heron uses [gitflow](https://nvie.com/posts/a-successful-git-branching-model/).
Two branches live permanently; everything else is short-lived and merged by
pull request.

| Branch | Purpose | Branches from | Merges into |
|---|---|---|---|
| `develop` | Integration: the latest finished work. The default branch. | — | — |
| `main` | Releases only. Every commit on it is a tagged release. | — | — |
| `feature/<name>` | New work | `develop` | `develop` |
| `bugfix/<issue>` | A fix for something on `develop` | `develop` | `develop` |
| `chore/<name>` | Upkeep that changes no behaviour: docs, build and tooling setup, dependency bumps, cleanups | `develop` | `develop` |
| `release/<version>` | Preparing a release | `develop` | `main` and `develop` |
| `hotfix/<issue>` | An urgent fix to a release | `main` | `main` and `develop` |

**Never commit directly to `develop` or `main`.** Every change reaches them
through a pull request from one of the branches above.

### Naming

- Lowercase, words separated by hyphens: `feature/wheel-odometry`, not
  `feature/WheelOdometry` or `feature/wheel_odometry`.
- `feature/` and `chore/` are singular.
- `chore/` names the change, like `feature/`, and needs no issue:
  `chore/update-doc-to-fix-venv`. If the change fixes behaviour, it is a
  `bugfix/` instead, even when it is small.
- `bugfix/` and `hotfix/` start with the GitHub issue number, optionally
  followed by a short description: `bugfix/12` or `bugfix/12-gui-reconnect`.
  Open an issue first if there is none.
- `release/` carries the version without a `v`: `release/0.1.0`.

## Making a change

```
git switch develop
git pull
git switch -c feature/wheel-odometry      # or bugfix/12-gui-reconnect, chore/...

# ... work, committing as you go ...

git push -u origin feature/wheel-odometry
```

Then open a pull request **into `develop`** (it is the default, so GitHub
targets it unless told otherwise). Reference the issue in the description
(`Fixes #12`) so it closes on merge. Delete the branch once it is merged.

Keep a branch to one feature or one fix. Rebase it onto `develop`, or merge
`develop` into it, if `develop` has moved on and the pull request conflicts.

## Before opening a pull request

- **It builds, without warnings,** for the host and, if it touches C++, for
  the Pi:

  ```
  cmake --preset host && cmake --build --preset host
  ctest --test-dir build-host
  scripts/build_cpp.sh
  ```

- **The tests pass,** including any you added. Code that touches the LiDAR
  library should also pass the recording checks (`lidar_gapcheck.py` and
  `lidar_adaptercheck.py`), which compare it point for point against the
  Python decoder. See [`docs/cpp-build.md`](docs/cpp-build.md).
- **Hardware claims are measured.** If the change depends on how a device
  behaves, run it on the hardware and say in the pull request what was
  measured and what was not. Unverified behaviour is labelled as unverified.
- **Docs are current.** A change to how something works updates the document
  that describes it, and work on a subsystem updates its resume file,
  `docs/RESUME-<area>.md`.

## Conventions

- **C++** follows [`CPP_CODING_STANDARD.md`](CPP_CODING_STANDARD.md). Every
  `.h` and `.cpp` file starts with the MIT file header given there. Standard
  library only.
- **Shell** scripts use `#!/usr/bin/env bash` and `set -Eeuo pipefail`, four
  spaces, `${VAR}` braces, `ERROR:`-prefixed messages, and comments that
  explain why. They must run on macOS's bash 3.2 and `openrsync`.
- **Python** that runs on the Pi uses only packages Debian ships, declared in
  `target/provision/base.sh`. Host-only dependencies go in
  `requirements-host.txt`.
- **The Pi is a target, not a workstation.** Nothing is edited, built or
  cloned on it; its configuration comes from the scripts in this repository.
- **Commit messages** have a short imperative subject, capitalized, with no
  issue numbers (those go in the pull request), and a body explaining why
  when it is not obvious. Commit in logical steps rather than one lump.

## Releases

Releases are cut from `develop` by the maintainer:

1. Branch `release/<version>` from `develop`. Only version bumps and fixes
   for the release go on it.
2. Merge it into `main`, and tag that merge `v<version>` (`v0.1.0`).
3. Merge it back into `develop`, so fixes made during the release are not
   lost. Delete the release branch.

A `hotfix/<issue>` branch works the same way, but starts from `main` and ends
with a patch version (`v0.1.1`).

Versions follow [Semantic Versioning](https://semver.org/). Until 1.0.0,
any release may change interfaces.

## Secrets

Never commit credentials, tokens, keys or a device's network address.
Deployment commands write the target as `user@<pi>`; keep real addresses
out of the repository. A [gitleaks](https://github.com/gitleaks/gitleaks)
pre-commit hook catches most mistakes before they become history:

```
brew install gitleaks
cat > .git/hooks/pre-commit <<'EOF'
#!/bin/sh
exec gitleaks git --pre-commit --staged --redact --no-banner -v
EOF
chmod +x .git/hooks/pre-commit
```

## License

Heron is MIT-licensed (see [`LICENSE`](LICENSE)). By contributing, you agree
that your contributions are licensed under the same terms.
