# tools/

Bench operation, not experiments. Nothing here produces a result that appears in
the paper — these exist so a hardware session does not get wasted on setup.

`scripts/` is the opposite: it is the experimental record, paired with
`results/`, and should not be tidied by deletion.

| tool | what it is for |
|---|---|
| `validate_record_args.py` | parses the real record config with the exact argv `arms.sh` passes, without opening a serial port or camera. Run it from the desk before a bench session; it is what caught the py3.10 `pyav_utils` crash. |
| `camera_stream.py` | serves the bench camera as MJPEG on a port, for checking framing from a browser over Tailscale. Holds the camera — stop it before teleop or record. |
| `export_episodes.py` | splits a recorded dataset's AV1 chunk into per-episode H.264 clips plus an index page, because lerobot writes one blob of all episodes in a codec Safari will not play. |

```bash
python tools/validate_record_args.py
python tools/camera_stream.py                     # then open http://<host>:8000/
python tools/export_episodes.py --root data/real/<name> --serve 8001
```
