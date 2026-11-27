#!/usr/bin/env python3
"""Loading test campaign for ROS1 bags and MCAP files in lidar_odometry_step_1.

Downloads public lidar + IMU recordings from Hugging Face and Zenodo, rewrites each one into every container and
compression step 1 reads (MCAP none/lz4/zstd, ROS1 bag none/bz2/lz4, and recordings split into several files), then
runs lidar_odometry_step_1 on every variant and times its loading. All variants of one recording must load the same
number of points and IMU samples; a difference is reported as a mismatch.

Run it through bag_loading_campaign.sh, which sets up a Python environment with rosbags, mcap, lz4 and zstandard.

Steps (default: all of them):
  download  fetch the recordings into <data>/downloads (resumes partial downloads)
  prepare   write the variants into <data>/variants/<recording>/<lidar>/<variant>/
  run       load every variant with lidar_odometry_step_1, write <data>/results.csv and <data>/results.md
"""

from __future__ import annotations

import argparse
import csv
import datetime
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class Recording:
    """A public recording with a lidar and an IMU topic."""

    name: str
    url: str
    size_mb: int
    license: str
    note: str
    archive: bool = False  # a .tar.gz holding several bags; the smallest one is used
    large: bool = False  # downloaded only with --large
    imu_topic: str = ""  # empty: the IMU topic with the most messages


RECORDINGS = [
    Recording(
        name="fastlivo_hku2",
        url="https://huggingface.co/datasets/DapengFeng/MCAP/resolve/main/FAST-LIVO/hku2/hku2_0.mcap",
        size_mb=885,
        license="CC-BY-NC-4.0",
        note="FAST-LIVO hku2 as ROS 2 MCAP (CDR): Livox Avia livox_ros_driver/CustomMsg + IMU, 105 s",
    ),
    Recording(
        name="koide_livox_ros1",
        url="https://zenodo.org/api/records/7780490/files/livox_ros1.tar.gz/content",
        size_mb=1945,
        license="CC-BY-4.0",
        note="Koide LiDAR-camera calibration dataset (Zenodo 7780490), ROS1 bag: Livox Avia PointCloud2 and CustomMsg + IMU",
        archive=True,
    ),
    Recording(
        name="hilti2022_exp14",
        url="https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/rosbags/exp14_basement_2.bag",
        size_mb=6260,
        license="CC-BY-NC-SA-3.0",
        note="Hilti SLAM Challenge 2022 exp14, ROS1 bag: Hesai PandarXT-32 PointCloud2 + IMU, cameras dropped",
        large=True,
    ),
]

LIDAR_TYPES = {"PointCloud2": "pc2", "CustomMsg": "custommsg"}
SPLIT_PARTS = 3


# --------------------------------------------------------------------------------------------------------------------
# download


def log(message: str) -> None:
    print(f"[{datetime.datetime.now():%H:%M:%S}] {message}", flush=True)


def download(url: str, target: Path) -> None:
    """Downloads url to target, resuming a partial file; a complete file is kept."""
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + ".part")
    if target.exists():
        log(f"{target.name}: already downloaded")
        return
    offset = partial.stat().st_size if partial.exists() else 0
    request = urllib.request.Request(url, headers={"User-Agent": "hdmapping-bag-campaign"})
    if offset:
        request.add_header("Range", f"bytes={offset}-")
    with urllib.request.urlopen(request) as response:
        if offset and response.status != 206:
            offset = 0  # the server ignored the range: start over
        total = offset + int(response.headers.get("Content-Length", 0))
        done = offset
        last = 0.0
        with open(partial, "ab" if offset else "wb") as out:
            while chunk := response.read(1 << 20):
                out.write(chunk)
                done += len(chunk)
                if time.monotonic() - last > 5:
                    last = time.monotonic()
                    percent = f"{100 * done / total:.0f}%" if total else "?"
                    log(f"{target.name}: {done >> 20} / {total >> 20} MB ({percent})")
    partial.rename(target)
    log(f"{target.name}: downloaded {done >> 20} MB")


def source_file(recording: Recording, data: Path) -> Path:
    """The downloaded recording, extracted from its archive when it is one."""
    downloads = data / "downloads" / recording.name
    if not recording.archive:
        return downloads / Path(recording.url).name
    extracted = downloads / "extracted"
    bags = sorted(extracted.rglob("*.bag"), key=lambda p: p.stat().st_size) if extracted.exists() else []
    if not bags:
        raise RuntimeError(f"{recording.name}: no .bag in {extracted}; run the download step")
    return bags[0]


def download_recording(recording: Recording, data: Path) -> None:
    downloads = data / "downloads" / recording.name
    if not recording.archive:
        download(recording.url, downloads / Path(recording.url).name)
        return
    archive = downloads / f"{recording.name}.tar.gz"
    extracted = downloads / "extracted"
    if extracted.exists() and any(extracted.rglob("*.bag")):
        log(f"{recording.name}: already extracted")
        return
    download(recording.url, archive)
    log(f"{recording.name}: extracting {archive.name}")
    with tarfile.open(archive) as tar:
        tar.extractall(extracted, filter="data")


# --------------------------------------------------------------------------------------------------------------------
# prepare


@dataclass
class Topics:
    imu: str
    lidars: dict[str, str] = field(default_factory=dict)  # label (pc2, custommsg) -> topic
    start_ns: int = 0
    end_ns: int = 0


def find_topics(path: Path, imu_topic: str) -> Topics:
    from rosbags.highlevel import AnyReader

    with AnyReader([path]) as reader:
        imus = [c for c in reader.connections if c.msgtype.endswith("/Imu")]
        if imu_topic:
            imus = [c for c in imus if c.topic == imu_topic]
        if not imus:
            raise RuntimeError(f"{path}: no sensor_msgs/Imu topic")
        imu = max(imus, key=lambda c: c.msgcount)
        topics = Topics(imu=imu.topic, start_ns=reader.start_time, end_ns=reader.end_time)
        for c in sorted(reader.connections, key=lambda c: -c.msgcount):
            label = LIDAR_TYPES.get(c.msgtype.rsplit("/", 1)[-1])
            if label and label not in topics.lidars:
                topics.lidars[label] = c.topic
    if not topics.lidars:
        raise RuntimeError(f"{path}: no PointCloud2 or CustomMsg topic")
    return topics


def rosbags_convert(sources: list[Path], dst: Path, *args: str) -> None:
    """python -m rosbags.convert; dst ending in .bag is a ROS1 bag, otherwise a rosbag2 folder."""
    command = [sys.executable, "-m", "rosbags.convert", "--src", *map(str, sources), "--dst", str(dst), *args]
    subprocess.run(command, check=True)


def to_mcap(sources: list[Path], target: Path, topics: list[str]) -> None:
    """The topics of sources as one uncompressed ROS 2 MCAP file at target."""
    folder = target.with_suffix(".rosbag2")
    shutil.rmtree(folder, ignore_errors=True)
    rosbags_convert(sources, folder, "--dst-storage", "mcap", "--include-topic", *topics)
    (mcap,) = folder.glob("*.mcap")
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(mcap), target)
    shutil.rmtree(folder)


def rewrite_mcap(source: Path, target: Path, compression: str, start_ns: int = 0, end_ns: int | None = None) -> None:
    """Copies the messages of source with log time in [start_ns, end_ns) into target with this chunk compression."""
    from mcap.reader import make_reader
    from mcap.writer import CompressionType, Writer

    target.parent.mkdir(parents=True, exist_ok=True)
    with open(source, "rb") as src, open(target, "wb") as out:
        reader = make_reader(src)
        header = reader.get_header()
        writer = Writer(out, compression=CompressionType[compression.upper()])
        writer.start(profile=header.profile, library="hdmapping-bag-campaign")
        schemas: dict[int, int] = {}
        channels: dict[int, int] = {}
        for schema, channel, message in reader.iter_messages(start_time=start_ns, ending_before=end_ns, log_time_order=True):
            if schema and schema.id not in schemas:
                schemas[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
            if channel.id not in channels:
                channels[channel.id] = writer.register_channel(
                    channel.topic, channel.message_encoding, schemas.get(channel.schema_id, 0), channel.metadata
                )
            writer.add_message(channels[channel.id], message.log_time, message.data, message.publish_time, message.sequence)
        writer.finish()


def prepare_recording(recording: Recording, data: Path) -> None:
    source = source_file(recording, data)
    topics = find_topics(source, recording.imu_topic)
    log(f"{recording.name}: IMU {topics.imu}, lidar {topics.lidars}")
    for label, lidar in topics.lidars.items():
        base = data / "variants" / recording.name / label
        if (base / ".complete").exists():
            log(f"{recording.name}/{label}: variants already prepared")
            continue
        shutil.rmtree(base, ignore_errors=True)
        name = recording.name
        plain = base / "mcap_none" / f"{name}.mcap"
        log(f"{recording.name}/{label}: MCAP, uncompressed")
        to_mcap([source], plain, [lidar, topics.imu])
        for compression in ("lz4", "zstd"):
            log(f"{recording.name}/{label}: MCAP, {compression}")
            rewrite_mcap(plain, base / f"mcap_{compression}" / f"{name}.mcap", compression)
        for compression in ("none", "bz2", "lz4"):
            log(f"{recording.name}/{label}: ROS1 bag, {compression}")
            target = base / f"ros1_{compression}" / f"{name}.bag"
            target.parent.mkdir(parents=True, exist_ok=True)
            rosbags_convert([plain], target, *([] if compression == "none" else ["--compress", compression]))

        # one recording in several files, the last part in a subfolder: dropped folders are searched recursively
        log(f"{recording.name}/{label}: split into {SPLIT_PARTS} files (MCAP zstd, ROS1 bz2)")
        step = (topics.end_ns - topics.start_ns) // SPLIT_PARTS + 1
        for k in range(SPLIT_PARTS):
            sub = f"part_{k}" if k == SPLIT_PARTS - 1 else ""
            start = topics.start_ns + k * step
            part = base / f"split{SPLIT_PARTS}_mcap_zstd" / sub / f"{name}_{k}.mcap"
            rewrite_mcap(plain, part, "zstd", start, start + step)
            bag = base / f"split{SPLIT_PARTS}_ros1_bz2" / sub / f"{name}_{k}.bag"
            bag.parent.mkdir(parents=True, exist_ok=True)
            rosbags_convert([part], bag, "--compress", "bz2")
        (base / ".complete").write_text(f"lidar {lidar}\nimu {topics.imu}\nsource {source}\n")


# --------------------------------------------------------------------------------------------------------------------
# run


@dataclass
class Result:
    recording: str
    lidar: str
    variant: str
    files: int
    size_mb: float
    imu_s: float | None = None
    points_s: float | None = None
    total_s: float | None = None
    imu_samples: int | None = None
    points: int | None = None
    status: str = ""


def variant_input(folder: Path) -> tuple[Path, int, float]:
    """The path step 1 opens for a variant (the file, or the folder of a split recording), its file count and size."""
    files = sorted(p for p in folder.rglob("*") if p.suffix in (".bag", ".mcap"))
    size = sum(p.stat().st_size for p in files) / 1e6
    return (files[0] if len(files) == 1 else folder), len(files), size


def campaign_params(step1: Path, data: Path) -> Path:
    """Step 1's default parameters with skip_first_file off: with it on, step 1 drops the first chunk's points, and a
    recording split into files shorter than a chunk drops a different share, so the point counts would not compare."""
    params = data / "params_campaign.toml"
    subprocess.run([str(step1), "--dump-default-params", str(params)], check=True, stdout=subprocess.DEVNULL)
    text = params.read_text(encoding="utf-8")
    if "skip_first_file = true" not in text:
        raise RuntimeError(f"{params}: no 'skip_first_file = true' to turn off")
    params.write_text(text.replace("skip_first_file = true", "skip_first_file = false"), encoding="utf-8")
    return params


def run_step1(step1: Path, params: Path | None, folder: Path, timeout_s: float, full: bool, logs: Path) -> Result:
    recording, lidar, variant = folder.parts[-3:]
    path, files, size = variant_input(folder)
    result = Result(recording, lidar, variant, files, round(size, 1))
    before = set(folder.iterdir())
    log_file = logs / f"{recording}__{lidar}__{variant}.log"
    log(f"{recording}/{lidar}/{variant}: loading {files} file(s), {size:.0f} MB")

    start = time.monotonic()
    with open(log_file, "w", encoding="utf-8", errors="replace") as out:
        # with a parameter file step 1 also needs an output folder; it is inside the variant's folder, removed below
        command = [str(step1), str(path)] + ([str(params), str(folder / "campaign_output")] if params else [])
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            cwd=folder,
        )
        assert process.stdout
        try:
            for line in process.stdout:
                out.write(line)
                now = time.monotonic() - start
                if result.imu_s is None and line.startswith("Loading point clouds"):
                    result.imu_s = round(now, 2)
                if match := re.match(r"IMU samples: (\d+)", line):
                    result.imu_samples = int(match.group(1))
                if match := re.match(r"TOTAL: (\d+)", line):
                    result.points = int(match.group(1))
                if "loading finished" in line:
                    result.total_s = round(now, 2)
                    if result.imu_s is not None:
                        result.points_s = round(result.total_s - result.imu_s, 2)
                    if not full:
                        break
                if now > timeout_s:
                    result.status = f"timeout after {timeout_s:.0f} s"
                    break
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()
    if full and not result.status:
        result.status = f"exit {process.returncode} after {time.monotonic() - start:.0f} s"
    elif not result.status:
        result.status = "loaded" if result.total_s is not None else f"failed (exit {process.returncode}), see {log_file.name}"

    if not full:  # the run's lio_result_*, preview/ and cache/
        for entry in set(folder.iterdir()) - before:
            shutil.rmtree(entry, ignore_errors=True) if entry.is_dir() else entry.unlink(missing_ok=True)
    log(f"  {result.status}: total {result.total_s} s (IMU {result.imu_s} s), {result.points} points, {result.imu_samples} IMU samples")
    return result


def check_consistency(results: list[Result]) -> list[str]:
    """Variants of one recording and lidar topic that load different point or IMU sample counts."""
    problems = []
    groups: dict[tuple[str, str], list[Result]] = {}
    for r in results:
        groups.setdefault((r.recording, r.lidar), []).append(r)
    for (recording, lidar), group in groups.items():
        for key in ("points", "imu_samples"):
            values = {getattr(r, key) for r in group if r.status == "loaded"}
            if len(values) > 1:
                detail = ", ".join(f"{r.variant}={getattr(r, key)}" for r in group if r.status == "loaded")
                problems.append(f"{recording}/{lidar}: {key} differ: {detail}")
    return problems


def write_results(results: list[Result], problems: list[str], data: Path, step1: Path, params: Path | None) -> None:
    with open(data / "results.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(Result.__dataclass_fields__.keys())
        for r in results:
            writer.writerow(r.__dict__.values())
    lines = [
        f"# Bag loading campaign, {datetime.datetime.now():%Y-%m-%d %H:%M}",
        "",
        f"- step 1: `{step1}`" + ("" if params else " (default parameters)"),
        f"- machine: {platform.platform()}, {os.cpu_count()} logical CPUs",
        "",
        "| recording | lidar | variant | files | MB | IMU s | points s | total s | points | IMU samples | status |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for r in results:
        lines.append(
            f"| {r.recording} | {r.lidar} | {r.variant} | {r.files} | {r.size_mb} | {r.imu_s} | {r.points_s} | {r.total_s} "
            f"| {r.points} | {r.imu_samples} | {r.status} |"
        )
    lines += ["", "## Consistency", ""] + ([f"- {p}" for p in problems] or ["- every variant loads the same data"])
    (data / "results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


def find_step1(explicit: str | None) -> Path:
    if explicit:  # absolute: step 1 runs in each variant's folder
        return Path(explicit).resolve()
    repository = Path(__file__).resolve().parent.parent
    names = ["lidar_odometry_step_1.exe", "lidar_odometry_step_1"]
    for folder in ["build/bin", "build/bin/Release", "build/bin/RelWithDebInfo", "build/Release", "build"]:
        for name in names:
            candidate = repository / folder / name
            if candidate.is_file():
                return candidate
    on_path = shutil.which("lidar_odometry_step_1")
    if on_path:
        return Path(on_path)
    raise SystemExit("lidar_odometry_step_1 not found; pass --step1 <path to the executable>")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("steps", nargs="*", help="download, prepare, run (default: all three)")
    parser.add_argument("--data", default=str(Path.home() / "hdmapping_bag_campaign"), help="working folder (default: %(default)s)")
    parser.add_argument("--step1", help="lidar_odometry_step_1 executable (default: found in build/)")
    parser.add_argument("--large", action="store_true", help="include the large recordings (Hilti 2022, 6.3 GB download)")
    parser.add_argument("--only", nargs="+", help="only these recordings (names as in --list)")
    parser.add_argument("--variants", nargs="+", help="only these variants, e.g. ros1_bz2 mcap_zstd")
    parser.add_argument("--full", action="store_true", help="let step 1 process every variant to the end instead of stopping after loading")
    parser.add_argument(
        "--default-params", action="store_true", help="run step 1 with its defaults (skip_first_file on: split recordings then load fewer points)"
    )
    parser.add_argument("--timeout", type=float, default=3600, help="seconds one variant may take (default: %(default)s)")
    parser.add_argument("--list", action="store_true", help="list the recordings and exit")
    args = parser.parse_args()

    recordings = [r for r in RECORDINGS if (args.large or not r.large) and (not args.only or r.name in args.only)]
    if args.list:
        for r in RECORDINGS:
            print(f"{r.name:20} {r.size_mb:6} MB  {r.license:16} {'(--large) ' if r.large else ''}{r.note}\n{'':20} {r.url}")
        return 0
    steps = args.steps or ["download", "prepare", "run"]
    if unknown := set(steps) - {"download", "prepare", "run"}:
        parser.error(f"unknown step(s): {', '.join(sorted(unknown))}")
    data = Path(args.data).resolve()
    data.mkdir(parents=True, exist_ok=True)

    if "download" in steps:
        for r in recordings:
            download_recording(r, data)
    if "prepare" in steps:
        for r in recordings:
            prepare_recording(r, data)
    if "run" in steps:
        step1 = find_step1(args.step1)
        params = None if args.default_params else campaign_params(step1, data)
        logs = data / "logs"
        logs.mkdir(exist_ok=True)
        results = []
        for r in recordings:
            for lidar in sorted((data / "variants" / r.name).glob("*")):
                for folder in sorted(p for p in lidar.iterdir() if p.is_dir()):
                    if args.variants and folder.name not in args.variants:
                        continue
                    results.append(run_step1(step1, params, folder, args.timeout, args.full, logs))
        problems = check_consistency(results)
        write_results(results, problems, data, step1, params)
        return 1 if problems or any(r.status not in ("loaded",) and not args.full for r in results) else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
