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
