#!/usr/bin/env python3
"""
Walk folder(s) or single video file(s) on disk (e.g. zNAS), find .mp4/.mov, extract embedded
timed GPS via exiftool -ee, write sidecar <stem>_gps.csv and <stem>_gps.gpx next to each video.

Requires: exiftool on PATH (brew install exiftool).
GPX times use the camera's GPSDateTime string with a Z suffix (UTC as stored by exiftool).
"""
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import xml.sax.saxutils as xml_esc
from pathlib import Path


VIDEO_SUFFIXES = {".mp4", ".mov", ".MP4", ".MOV"}


def exiftool_rows(video: Path) -> list[tuple[float, float, float, float, str]]:
    """Returns list of (sampletime_s, lat, lon, alt_m, gps_datetime_raw)."""
    cmd = [
        "exiftool",
        "-ee",
        "-n",
        "-p",
        "$sampletime,$gpslatitude,$gpslongitude,$gpsaltitude,$gpsdatetime",
        str(video),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(f"{video}: exiftool failed ({proc.returncode})\n{proc.stderr}")
        return []
    rows: list[tuple[float, float, float, float, str]] = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("Warning:"):
            continue
        parts = line.split(",", 4)
        if len(parts) != 5:
            continue
        try:
            st = float(parts[0])
            lat = float(parts[1])
            lon = float(parts[2])
            alt = float(parts[3])
        except ValueError:
            continue
        dt = parts[4].strip()
        if not dt:
            continue
        rows.append((st, lat, lon, alt, dt))
    return rows


def gps_datetime_to_iso_z(s: str) -> str | None:
    """exiftool: '2026:05:12 09:58:59' -> ISO 8601 with Z (treat as UTC)."""
    s = s.strip()
    if " " not in s:
        return None
    date_part, time_part = s.split(" ", 1)
    if date_part.count(":") != 2:
        return None
    y, mo, d = date_part.split(":")
    return f"{y}-{mo}-{d}T{time_part.strip()}Z"


def write_csv(path: Path, rows: list[tuple[float, float, float, float, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["sample_time_s", "latitude", "longitude", "altitude_m", "gps_datetime"])
        for r in rows:
            w.writerow(r)


def write_gpx(path: Path, video: Path, rows: list[tuple[float, float, float, float, str]]) -> None:
    name = xml_esc.escape(video.name)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<gpx version="1.1" creator="dji_folder_gps_export.py" '
        'xmlns="http://www.topografix.com/GPX/1/1">',
        "<trk>",
        f"<name>{name}</name>",
        "<trkseg>",
    ]
    for _st, lat, lon, alt, dt_raw in rows:
        t = gps_datetime_to_iso_z(dt_raw)
        if t is None:
            continue
        lines.append(
            f'  <trkpt lat="{lat:.7f}" lon="{lon:.7f}">'
            f"<ele>{alt:.1f}</ele><time>{t}</time></trkpt>"
        )
    lines.extend(["</trkseg>", "</trk>", "</gpx>"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def iter_videos(root: Path, recursive: bool) -> list[Path]:
    if root.is_file():
        return [root] if root.suffix in VIDEO_SUFFIXES else []
    if recursive:
        out: list[Path] = []
        for p in root.rglob("*"):
            if p.is_file() and p.suffix in VIDEO_SUFFIXES:
                out.append(p)
        return sorted(out)
    return sorted(p for p in root.iterdir() if p.is_file() and p.suffix in VIDEO_SUFFIXES)


def main() -> None:
    ap = argparse.ArgumentParser(description="Export DJI embedded GPS (exiftool -ee) to CSV+GPX.")
    ap.add_argument(
        "paths",
        nargs="+",
        type=Path,
        help="Folder(s) to scan and/or individual .mp4/.mov files",
    )
    ap.add_argument("--no-recursive", action="store_true", help="Only files directly in each folder (default: recurse)")
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="List matching video paths only (no exiftool; fast for huge NAS trees)",
    )
    ap.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Stop after this many video files examined (0 = no limit). Counts every candidate.",
    )
    args = ap.parse_args()
    recursive = not args.no_recursive

    total = 0
    exported = 0
    skipped = 0
    stop = False
    for folder in args.paths:
        if stop:
            break
        if not folder.exists():
            sys.stderr.write(f"Missing path: {folder}\n")
            continue
        if folder.is_file():
            if folder.suffix not in VIDEO_SUFFIXES:
                sys.stderr.write(f"Not a video file: {folder}\n")
                continue
        elif not folder.is_dir():
            sys.stderr.write(f"Not a directory or video file: {folder}\n")
            continue
        for vid in iter_videos(folder, recursive):
            if args.limit and total >= args.limit:
                stop = True
                break
            total += 1
            if args.dry_run:
                print(vid)
                continue
            rows = exiftool_rows(vid)
            if not rows:
                skipped += 1
                continue
            csv_p = vid.with_name(vid.stem + "_gps.csv")
            gpx_p = vid.with_name(vid.stem + "_gps.gpx")
            write_csv(csv_p, rows)
            write_gpx(gpx_p, vid, rows)
            print(f"{vid} -> {csv_p.name} ({len(rows)} pts), {gpx_p.name}")
            exported += 1
    if args.dry_run:
        print(f"done: videos_matched={total}")
    else:
        print(f"done: videos_seen={total} exported={exported} no_embedded_gps={skipped}")


if __name__ == "__main__":
    main()
