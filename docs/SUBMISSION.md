# Submission — Team 6131

**Authors:** Agrim Chauhan, Saksham Kaushal, Rayhan Khan, Manan Sood

## Prepared archives

| Task | Archive in `~/ros2_ws/submissions/` | Saved evaluator score |
| --- | --- | --- |
| 2A | `SC#6131_task2A.zip` | 50/50 |
| 2B | `SC#6131_task2B.zip` | 50/50 |

Each ZIP contains exactly three files at its root:

```text
SC#6131_task2A.zip          SC#6131_task2B.zip
├── result.zip             ├── result.zip
├── task2A_perception.py    ├── task2B_planning.py
└── task2A_manipulation.py  └── task2B_navigation.py
```

The inner `result.zip` files are the original signed evaluator outputs. They have
not been unpacked, modified or rebuilt. The surrounding submission ZIPs were updated
with team headers on 21 October 2026. Python syntax trees were compared before and
after formatting to confirm that the evaluated implementations are unchanged.

The original outer archives are retained locally under
`results/task2a/submission-original/` and `results/task2b/submission-original/`. The
`results/` and `submissions/` folders are excluded from Git, so the signed result files
remain in the local workspace and are not uploaded to this repository.

## Source versions

The Task 2A submission scripts match the current workspace scripts. Task 2B retains
the planner from the recorded 50/50 evaluation. The workspace planner additionally
has two later corrections: retrying an unreachable speculative search connection,
and checking grid cells incident to a path endpoint on a cell boundary. The Task 2B
navigation script matches the workspace version.

For a new evaluation, build and run the workspace scripts, then package that new
result with those same scripts. Keep the preserved 50/50 archive paired with its
evaluated planner.

## Recording and portal upload

The saved validation runs do not establish that a matching submission video exists.
The earlier Task 2A validation notes explicitly record headless runs without a video.
The portal requires the recording and unlisted video URL for the same run as the
submitted result. Record a fresh GUI run if that matching video is unavailable.

Upload only the task's submission ZIP and the required video URL. The full workspace,
historical results, build directories and documentation do not belong inside that ZIP.

## Package a new run

After scoring, keep the scripts that produced that run unchanged. Set `task` to `2A`
or `2B` below. This writes a new archive under `submissions/new-run/`, preserving the
existing prepared archives:

```bash
python3 - <<'PY'
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

task = '2B'
root = Path.home() / 'ros2_ws'
names = {
    '2A': ('task2A_perception.py', 'task2A_manipulation.py'),
    '2B': ('task2B_planning.py', 'task2B_navigation.py'),
}[task]
result = root / 'src' / f'SC-6131-task-{task}' / 'result.zip'
scripts = root / 'src/algorithms/scripts' / f'task{task.lower()}'
files = [result, *(scripts / name for name in names)]
for path in files:
    if not path.is_file():
        raise SystemExit(f'Missing file: {path}')
output = root / 'submissions/new-run' / f'SC#6131_task{task}.zip'
output.parent.mkdir(parents=True, exist_ok=True)
with ZipFile(output, 'x', ZIP_DEFLATED) as archive:
    for path in files:
        archive.write(path, arcname=path.name)
print(output)
PY
```

This copies the signed `result.zip` as an opaque file. Preserve each completed run
before another evaluator invocation overwrites its output directory.

Task 1 result files and the Task 1A detection image are retained in `results/task1a/`,
`results/task1b/` and `results/task1c/`. Their task-specific portal submission formats
should be followed when preparing those earlier tasks.
