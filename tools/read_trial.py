#!/usr/bin/env python3
"""Print what a recorded run actually showed: which stimulus was on which side.

    python3 tools/read_trial.py /data/mosquito/2026-09-28/trial_20260928_101500
    python3 tools/read_trial.py <run>/assay          # the bag directly
    python3 tools/read_trial.py <run> --json         # machine-readable

Reads the bag's SQLite store directly, so it needs no ROS environment and no
`ros2 bag play`. (`ros2 topic echo --from-bag` does not exist in Humble.)

Which stimulus went left and which went right is decided at run time by the
trial RNG, so it is not recorded in any config file -- it lives in the bag, in
the `left` and `right` objects of the trial_start message. This prints them,
along with the detection event that started the trial.
"""

import argparse
import glob
import json
import os
import sqlite3
import sys


def _decode(blob):
    """std_msgs/String out of a CDR blob: 4-byte encapsulation header, then a
    4-byte length, then the bytes, then a NUL."""
    try:
        return blob[8:].decode("utf-8", "ignore").rstrip("\x00")
    except Exception:                                   # noqa: BLE001
        return ""


def read_bag(path):
    """Yield (topic, parsed_json, timestamp_ns) for every JSON message."""
    dbs = sorted(glob.glob(os.path.join(path, "*.db3")))
    if not dbs:
        raise SystemExit(f"no .db3 file in {path} -- is that a bag directory?")
    for db in dbs:
        con = sqlite3.connect(db)
        topics = {i: n for i, n in con.execute("select id, name from topics")}
        rows = con.execute(
            "select topic_id, data, timestamp from messages order by timestamp")
        for topic_id, blob, stamp in rows:
            text = _decode(blob)
            if not text.startswith("{"):
                continue
            try:
                yield topics[topic_id], json.loads(text), stamp
            except (ValueError, KeyError):
                continue
        con.close()


def summarize(run_dir):
    bag = run_dir
    if os.path.isdir(os.path.join(run_dir, "assay")):
        bag = os.path.join(run_dir, "assay")

    trials, detections, experiment = [], [], None
    for topic, msg, _ in read_bag(bag):
        if topic.endswith("/trial_start"):
            trials.append(msg)
        elif topic.endswith("/experiment_info") and experiment is None:
            experiment = msg
        elif msg.get("event") == "mosquito_detected":
            detections.append(msg)

    return {
        "bag": bag,
        "experiment": experiment,
        "detections": detections,
        # phase "complete" repeats the trial; keep only where it started
        "trials": [t for t in trials if t.get("phase") == "running"],
        "video_bag": (os.path.join(run_dir, "video")
                      if os.path.isdir(os.path.join(run_dir, "video")) else None),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", help="a run folder, or the assay bag inside it")
    ap.add_argument("--json", action="store_true", help="dump the raw objects")
    args = ap.parse_args()

    if not os.path.isdir(args.run_dir):
        sys.exit(f"not a directory: {args.run_dir}")

    out = summarize(args.run_dir)
    if args.json:
        print(json.dumps(out, indent=2))
        return

    info = out["experiment"] or {}
    # experiment_info nests the experiment's own fields under "experiment";
    # master_seed and start_mode sit at the top level beside it.
    exp = info.get("experiment", {})
    print(f"bag        : {out['bag']}")
    if out["video_bag"]:
        print(f"video      : {out['video_bag']}")
    if info:
        print(f"experiment : {exp.get('name')}  (sha1 {str(exp.get('sha1'))[:12]})")
        print(f"master_seed: {info.get('master_seed')}"
              f"    start_mode: {info.get('start_mode')}")

    if not out["trials"]:
        print("\nNO TRIAL RAN in this bag -- the trigger never fired.")
        return

    for t in out["trials"]:
        left, right = t.get("left", {}), t.get("right", {})
        geo = t.get("geometry", {})
        print(f"\ntrial {t.get('trial_id')}  "
              f"condition {t.get('condition', {}).get('name')}")
        print(f"  LEFT   {left.get('name'):<16} ({left.get('type')})")
        print(f"  RIGHT  {right.get('name'):<16} ({right.get('type')})")
        print(f"  duration    {t.get('trial_duration_sec')} s"
              f"    trial_seed {t.get('trial_seed')}")
        elapsed = t.get("stimulus_elapsed_at_trial_start")
        if elapsed is not None:
            print(f"  stimuli had been playing {elapsed:.1f} s when the trial started")
        print(f"  centres     L{geo.get('left_center_px')} "
              f"R{geo.get('right_center_px')}  "
              f"diameter {geo.get('circle_diameter_px')}")

    for d in out["detections"]:
        pos = d.get("position_px", [])
        print(f"\ntriggered by a blob at "
              f"({pos[0]:.0f},{pos[1]:.0f})" if len(pos) == 2 else "\ntriggered")
        print(f"  area {d.get('area_px')} px   camera {d.get('image_topic')}")
        print(f"  zone {d.get('roi_px')}")


if __name__ == "__main__":
    main()
