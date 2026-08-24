# lerobot local-patch snapshot

The bench does not use a stock lerobot. `/home/jetson3/projects/clean_env/lerobot`
is an upstream checkout on `main` carrying local modifications that were never
committed there, plus untracked helper scripts. If that tree is ever reset,
re-cloned, or `git checkout .`-ed, all of it is lost. This directory is the backup.

## Contents

| File | What it holds |
|---|---|
| `lerobot-base-commit.txt` | upstream commit the patch applies onto |
| `lerobot-local.patch` | `git diff` of 13 modified tracked files |
| `lerobot-untracked-scripts.tar.gz` | 10 untracked files, incl. `src/lerobot/policies/normalize.py` |

`normalize.py` lives *inside* the lerobot package but is untracked, so it is
carried in the tarball, not the patch — restoring only the patch leaves a
broken tree.

## Restore

```bash
cd /home/jetson3/projects/clean_env/lerobot
git checkout "$(cat /home/jetson3/projects/so101-bench/patches/lerobot-base-commit.txt)"
git apply /home/jetson3/projects/so101-bench/patches/lerobot-local.patch
tar xzf /home/jetson3/projects/so101-bench/patches/lerobot-untracked-scripts.tar.gz
```

## Refresh the snapshot after changing the lerobot tree

```bash
cd /home/jetson3/projects/clean_env/lerobot
git diff > /home/jetson3/projects/so101-bench/patches/lerobot-local.patch
git rev-parse HEAD > /home/jetson3/projects/so101-bench/patches/lerobot-base-commit.txt
```

## Notable patch: `datasets/pyav_utils.py`

Upstream `bd9619dfc` (user-provided video encoding parameters) calls
`num_val.is_integer()` where `num_val` may be a bare `int`. `int.is_integer()`
is Python 3.12+; this venv is 3.10, so **every** `lerobot-record` run died in
config parsing before opening the camera. Patched to `float(num_val).is_integer()`.

Upstream declares Python 3.12+ (`lerobot/CLAUDE.md`) while this venv is 3.10 —
so this class of breakage can recur on upstream pulls. A scan for the other
common 3.11/3.12-only constructs (`itertools.batched`, `typing.override`,
`StrEnum`, `datetime.UTC`, `tomllib`, `TaskGroup`, `except*`) found none as of
this commit.

## Notable patch: `common/control_utils.py` (keyboard control over SSH)

Upstream drives the record loop's arrow keys through **pynput**, whose X backend
needs the X **RECORD** extension. The bench is driven over SSH with X forwarding
to XQuartz, which advertises no RECORD — so pynput raised
`AttributeError: record_create_context` from inside its own listener thread,
where the caller cannot catch it. `is_headless()` did not help: it only checks
whether pynput *imports*, which it does.

Even with RECORD present, pynput reads the **X server's** keyboard, i.e. the
Mac's — not the SSH terminal where the operator is actually typing. So the whole
approach is wrong for this setup, not merely unavailable.

Replaced with `_TerminalKeyListener`, which reads the controlling terminal
directly (cbreak + `select`, ISIG left on so Ctrl-C still works, terminal
restored via `stop()` and an `atexit` hook so a crash cannot strand the shell).
Verified against a pty: right arrow, left arrow, bare ESC, and stray-key
rejection all behave.

The pynput fallback is **removed rather than retained**. Deciding whether a
given display could support it means connecting through `python-xlib`, and that
call was measured to block indefinitely against an unreachable forwarded
display — a hang at record startup is worse than absent keys. With no TTY the
loop now warns and advances on the episode timer.
