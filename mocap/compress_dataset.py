"""
compress_dataset.py
--------------------
Compresses DNG frame sequences in a multi-camera mocap dataset, preserving the
full directory structure.  Two output formats are supported (--format):

  * mp4 (default): all DNGs sharing a directory are treated as one sequence
    (sorted by filename) and encoded into a single H.264 .mp4 inside the
    mirrored directory:
        libx264 -crf <crf> -preset <preset> -pix_fmt yuv420p -movflags +faststart
  * jpg: each DNG is converted to its own JPEG at the mirrored path.

All non-DNG files (XML, JSON, TXT, LOG, PNG …) are copied verbatim so the
compressed dataset remains self-contained.

Usage
-----
    # MP4 (default):
    python compress_dataset.py --src .../Source --dst .../Source_compressed \
        --crf 30 --fps 50

    # JPEG:
    python compress_dataset.py --src ... --dst ... --format jpg --jpeg-quality 90

    # Control parallelism (default: all CPU cores):
    python compress_dataset.py --src ... --dst ... --workers 8

    # Preview only (convert one sample and stop):
    python compress_dataset.py --src ... --dst ... --preview-only

Dependencies
------------
    pip install rawpy numpy pillow tqdm
    ffmpeg must be on PATH for --format mp4 (e.g. `sudo apt install ffmpeg`).
"""

import argparse
import multiprocessing as mp
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import rawpy
from PIL import Image
from tqdm import tqdm


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_dng_files(src_root: Path) -> list[Path]:
    """Return all .dng files under *src_root*, sorted for reproducibility."""
    return sorted(src_root.rglob("*.dng"))


def group_dng_by_folder(dng_files: list[Path]) -> dict[Path, list[Path]]:
    """Group DNG files by their parent directory; each group becomes one video."""
    groups: dict[Path, list[Path]] = defaultdict(list)
    for dng in dng_files:
        groups[dng.parent].append(dng)
    for frames in groups.values():
        frames.sort()
    return dict(groups)


def dng_to_rgb(src_path: Path) -> np.ndarray:
    """
    Decode *src_path* (DNG) to a contiguous uint8 RGB array with even
    dimensions (required by yuv420p / libx264).

    Single-channel (IR) cameras are replicated to three channels so every
    sequence produces a standard rgb24 stream.
    """
    with rawpy.imread(str(src_path)) as raw:
        # postprocess() applies the standard ISP pipeline: demosaic,
        # white-balance, colour-space conversion.  use_camera_wb=True honours
        # the camera's white-balance metadata recorded in the DNG.
        rgb = raw.postprocess(use_camera_wb=True, output_bps=8)

    if rgb.ndim == 2:                       # (H, W)        grayscale
        rgb = np.stack([rgb] * 3, axis=-1)
    elif rgb.shape[2] == 1:                 # (H, W, 1)     IR / mono
        rgb = np.repeat(rgb, 3, axis=2)

    # yuv420p requires even width and height; drop a trailing row/column if odd.
    h, w = rgb.shape[:2]
    rgb = rgb[: h - (h % 2), : w - (w % 2)]
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def dng_to_rgb_retry(src_path: Path, attempts: int = 4, delay: float = 3.0) -> np.ndarray:
    """
    Like dng_to_rgb, but retry transient OS errors (e.g. Errno 5 / EIO from a
    flaky network mount) a few times before giving up.
    """
    for attempt in range(attempts):
        try:
            return dng_to_rgb(src_path)
        except OSError:
            if attempt == attempts - 1:
                raise
            time.sleep(delay * (attempt + 1))
    raise AssertionError("unreachable")


def dng_to_jpeg(src_path: Path, dst_path: Path, quality: int) -> tuple[int, int]:
    """
    Convert a single DNG to a JPEG at *dst_path*; returns (src_bytes, dst_bytes).

    Writes to a temp file and atomically renames so an interrupted run never
    leaves a truncated .jpg that --resume would mistake for complete.
    """
    dst_path.parent.mkdir(parents=True, exist_ok=True)
    rgb = dng_to_rgb_retry(src_path)
    tmp_path = dst_path.with_name(dst_path.name + ".tmp")
    try:
        Image.fromarray(rgb).save(tmp_path, format="JPEG", quality=quality,
                                  subsampling=0)
        os.replace(tmp_path, dst_path)
    except BaseException:
        tmp_path.unlink(missing_ok=True)
        raise
    return src_path.stat().st_size, dst_path.stat().st_size


def _jpeg_worker(args: tuple) -> tuple[int, int, int, str | None, str]:
    """
    Picklable wrapper around dng_to_jpeg.  Returns
    (src_bytes, dst_bytes, n_frames, error, ident); a failure is a non-None
    *error* string instead of an exception, so one bad frame doesn't abort the
    whole pool.  KeyboardInterrupt still propagates.
    """
    src_path, dst_path, quality = args
    try:
        sb, db = dng_to_jpeg(src_path, dst_path, quality)
        return (sb, db, 1, None, str(dst_path))
    except Exception as exc:                            # noqa: BLE001
        return (0, 0, 0, f"{type(exc).__name__}: {exc}", str(src_path))


def encode_folder_to_mp4(
    frames: list[Path],
    dst_path: Path,
    fps: int,
    crf: int,
    preset: str,
) -> tuple[int, int, int]:
    """
    Encode an ordered list of DNG *frames* into a single H.264 MP4 at
    *dst_path* by piping decoded rgb24 frames to ffmpeg.

    Returns (src_bytes, dst_bytes, n_frames) for size-reporting.
    """
    dst_path.parent.mkdir(parents=True, exist_ok=True)

    # Decode the first frame to learn the (even) dimensions ffmpeg must expect.
    first = dng_to_rgb_retry(frames[0])
    h, w = first.shape[:2]

    # Encode to a temporary file and atomically rename on success.  This way an
    # interrupted or failed encode never leaves a corrupt, unplayable .mp4 (one
    # with frame data but no "moov" trailer) sitting at the final path.
    tmp_path = dst_path.with_name(dst_path.name + ".tmp")

    cmd = [
        "ffmpeg", "-y",
        "-loglevel", "error",
        # raw input stream description
        "-f", "rawvideo",
        "-pix_fmt", "rgb24",
        "-s", f"{w}x{h}",
        "-r", str(fps),
        "-i", "-",
        "-an",
        # One thread per ffmpeg: we already run one encode per CPU core across
        # folders, so letting x264 also multithread would oversubscribe the CPU
        # (cores² threads) and balloon memory via per-thread lookahead buffers.
        "-threads", "1",
        # encoding parameters (as requested)
        "-c:v", "libx264",
        "-crf", str(crf),
        "-preset", preset,
        "-pix_fmt", "yuv420p",
        "-movflags", "+faststart",
        "-f", "mp4",                 # temp file lacks an .mp4 extension
        str(tmp_path),
    ]

    src_bytes = 0
    # stderr → real temp file (not a PIPE) so ffmpeg can never block writing it
    # while we block writing stdin (classic pipe-buffer deadlock).
    with tempfile.TemporaryFile() as errfile:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=errfile,
        )
        assert proc.stdin is not None

        try:
            for i, dng in enumerate(frames):
                frame = first if i == 0 else dng_to_rgb_retry(dng)
                if frame.shape[:2] != (h, w):
                    # Force the reference size so every frame is exactly h*w*3
                    # bytes; a short/long write would desync the rawvideo stream
                    # and make ffmpeg abort (leaving no trailer).
                    frame = _fit_frame(frame, h, w)
                proc.stdin.write(frame.tobytes())
                try:
                    src_bytes += dng.stat().st_size
                except OSError:
                    pass  # size is only for reporting; don't fail the encode
            proc.stdin.close()
        except BrokenPipeError:
            pass  # ffmpeg exited early; the exit code / stderr below explain why
        except BaseException:
            # KeyboardInterrupt, decode errors, etc.: don't leave a half-written
            # temp file or a stray ffmpeg process behind.
            proc.kill()
            proc.wait()
            tmp_path.unlink(missing_ok=True)
            raise

        ret = proc.wait()
        if ret != 0:
            errfile.seek(0)
            err = errfile.read().decode(errors="replace")
            tmp_path.unlink(missing_ok=True)
            raise RuntimeError(f"ffmpeg failed for {dst_path} (exit {ret}):\n{err}")

    os.replace(tmp_path, dst_path)  # atomic within the same filesystem
    dst_bytes = dst_path.stat().st_size
    return src_bytes, dst_bytes, len(frames)


def _fit_frame(frame: np.ndarray, h: int, w: int) -> np.ndarray:
    """Crop or zero-pad *frame* to exactly (h, w, 3) so its byte size is fixed."""
    fh, fw = frame.shape[:2]
    out = np.zeros((h, w, 3), dtype=np.uint8)
    out[: min(h, fh), : min(w, fw)] = frame[: min(h, fh), : min(w, fw)]
    return out


def _encode_worker(args: tuple) -> tuple[int, int, int, str | None, str]:
    """
    Picklable wrapper around encode_folder_to_mp4 for multiprocessing.Pool.

    Returns (src_bytes, dst_bytes, n_frames, error, ident).  A failure is
    reported as a non-None *error* string (with zero byte counts) instead of
    raising, so one bad folder (e.g. a transient mount I/O error) doesn't abort
    the whole pool.  KeyboardInterrupt still propagates.
    args = (frames, dst_path, fps, crf, preset)
    """
    frames, dst_path, fps, crf, preset = args
    try:
        sb, db, n = encode_folder_to_mp4(frames, dst_path, fps, crf, preset)
        return (sb, db, n, None, str(dst_path))
    except Exception as exc:                            # noqa: BLE001
        return (0, 0, 0, f"{type(exc).__name__}: {exc}", str(dst_path))


def copy_non_dng(src_path: Path, dst_path: Path) -> None:
    """Copy a non-DNG file, creating parent directories as needed."""
    dst_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_path, dst_path)


def dst_path_for(src_file: Path, src_root: Path, dst_root: Path) -> Path:
    """Map a source path to its mirrored destination path."""
    rel = src_file.relative_to(src_root)
    return dst_root / rel


def dst_mp4_path(folder: Path, src_root: Path, dst_root: Path) -> Path:
    """
    Destination MP4 for a folder of DNG frames: the mirrored folder gains a
    single <foldername>.mp4 (one video per folder).
    """
    rel = folder.relative_to(src_root)
    return dst_root / rel / f"{folder.name}.mp4"


def dst_jpeg_path(src_dng: Path, src_root: Path, dst_root: Path) -> Path:
    """Mirror a DNG path to the destination with a .jpg extension."""
    rel = src_dng.relative_to(src_root).with_suffix(".jpg")
    return dst_root / rel


def fmt_size(n_bytes: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n_bytes < 1024:
            return f"{n_bytes:.1f} {unit}"
        n_bytes /= 1024
    return f"{n_bytes:.1f} TB"


def ensure_ffmpeg() -> None:
    """Exit with a helpful message if ffmpeg is not available on PATH."""
    if shutil.which("ffmpeg") is None:
        print(
            "Error: ffmpeg not found on PATH. Install it first, e.g.:\n"
            "    sudo apt install ffmpeg",
            file=sys.stderr,
        )
        sys.exit(1)


# ---------------------------------------------------------------------------
# Preview helpers
# ---------------------------------------------------------------------------

def probe_dimensions(dng: Path) -> tuple[int, int]:
    """Return the (even) output (width, height) of a DNG, matching the encoder."""
    with rawpy.imread(str(dng)) as raw:
        s = raw.sizes
    w, h = int(s.width), int(s.height)
    return w - (w % 2), h - (h % 2)


def has_ordering_risk(frames: list[Path]) -> bool:
    """
    True if filename (lexicographic) order may not match numeric/temporal order
    — e.g. unpadded names like frame2 sorting after frame10.

    *frames* is already sorted lexicographically; we extract the last integer in
    each stem and check that those integers are non-decreasing.
    """
    nums: list[int] = []
    for f in frames:
        matches = re.findall(r"\d+", f.stem)
        if not matches:
            return False  # no numbering to reason about
        nums.append(int(matches[-1]))
    return any(b < a for a, b in zip(nums, nums[1:]))


PREVIEW_MAX_FRAMES = 100  # cap the preview to the first N frames of one folder


def find_preview_folder(src_root: Path) -> tuple[Path, list[Path]] | None:
    """
    Return the first folder holding more than PREVIEW_MAX_FRAMES DNG files (so
    the preview is a full clip) plus its sorted frame paths.  Falls back to the
    largest DNG folder seen if none exceed the threshold; None if there are no
    DNGs.  Stops at the first qualifying folder, avoiding a full-tree scan.
    """
    best: tuple[int, Path, list[Path]] | None = None
    for dirpath, _dirnames, filenames in os.walk(src_root):
        dngs = sorted(
            Path(dirpath) / f for f in filenames if f.lower().endswith(".dng")
        )
        if not dngs:
            continue
        if len(dngs) > PREVIEW_MAX_FRAMES:
            return Path(dirpath), dngs
        if best is None or len(dngs) > best[0]:
            best = (len(dngs), Path(dirpath), dngs)
    return (best[1], best[2]) if best else None


def run_preview(src_root: Path, dst_root: Path, fmt: str, fps: int, crf: int,
                preset: str, jpeg_quality: int) -> None:
    """
    Convert a sample from one representative folder (a PREVIEW_MAX_FRAMES MP4
    clip, or a single JPEG), then extrapolate the full dataset size from the
    number of sibling folders at the same directory level (assuming one camera
    per folder with similar frame counts).
    """
    found = find_preview_folder(src_root)
    if found is None:
        print("No DNG files found under the source directory.")
        return
    folder, all_frames = found
    n_in_folder = len(all_frames)
    frames = all_frames[:PREVIEW_MAX_FRAMES]

    # Folders at the same level → estimate of the dataset's scale.
    sibling_dirs = [d for d in folder.parent.iterdir() if d.is_dir()]
    n_folders = len(sibling_dirs)
    est_total_frames = n_in_folder * n_folders

    try:
        w, h = probe_dimensions(frames[0])
        res = f"{w}×{h}"
    except Exception as exc:                           # noqa: BLE001
        res = f"unknown ({exc})"

    print("\n" + "=" * 60)
    print("  COMPRESSION PREVIEW")
    print("=" * 60)
    print(f"  Folder : {folder}")
    print(f"  Resol. : {res}")

    # ---- convert the sample -------------------------------------------------
    t0 = time.perf_counter()
    if fmt == "mp4":
        dst = dst_mp4_path(folder, src_root, dst_root)
        print(f"  Frames : {len(frames):,} of {n_in_folder:,} in folder")
        print(f"  Output : {dst}")
        print(f"  Encode : libx264 crf={crf} preset={preset} fps={fps}")
        if n_in_folder <= PREVIEW_MAX_FRAMES:
            print(f"  Note   : no folder exceeds {PREVIEW_MAX_FRAMES} frames; "
                  f"previewing the largest ({n_in_folder}).")
        if has_ordering_risk(all_frames):
            print("  ⚠  Filenames in this folder may sort out of temporal order "
                  "(unpadded numbering?).")
        print("-" * 60)
        src_bytes, dst_bytes, n_sample = encode_folder_to_mp4(
            frames, dst, fps, crf, preset)
        out_label = "MP4 size "
    else:
        dst = dst_jpeg_path(frames[0], src_root, dst_root)
        print(f"  Sample : 1 frame of {n_in_folder:,} in folder")
        print(f"  Output : {dst}")
        print(f"  Encode : JPEG quality={jpeg_quality}")
        print("-" * 60)
        src_bytes, dst_bytes = dng_to_jpeg(frames[0], dst, jpeg_quality)
        n_sample = 1
        out_label = "JPEG size"
    elapsed = time.perf_counter() - t0

    ratio = src_bytes / dst_bytes if dst_bytes else float("inf")
    print(f"  DNG size : {fmt_size(src_bytes)}")
    print(f"  {out_label}: {fmt_size(dst_bytes)}")
    print(f"  Ratio    : {ratio:.1f}×  (saved {100 - 100/ratio:.1f}%)")
    print(f"  Time     : {elapsed:.2f}s  ({n_sample / elapsed:.1f} frames/s)")
    print("=" * 60)

    # ---- extrapolate full dataset size from sibling-folder count ------------
    bytes_per_frame = dst_bytes / n_sample if n_sample else 0.0
    avg_dng_per_frame = src_bytes / n_sample if n_sample else 0.0
    est_compressed = bytes_per_frame * est_total_frames
    est_source = avg_dng_per_frame * est_total_frames
    overall = est_source / est_compressed if est_compressed else float("inf")

    print("\n" + "=" * 60)
    print("  ESTIMATED FULL DATASET (extrapolated)")
    print("=" * 60)
    print(f"  Folders at this level  : {n_folders:,}")
    print(f"  Frames per folder      : {n_in_folder:,}  (from preview folder)")
    print(f"  Estimated total frames : {est_total_frames:,}")
    print(f"  Per-frame {fmt} size     : {fmt_size(bytes_per_frame)}")
    print(f"  Estimated source (DNG) : ~{fmt_size(est_source)}")
    print(f"  Estimated compressed   : ~{fmt_size(est_compressed)}  "
          f"(→ ~{overall:.1f}× smaller)")
    print("  Note: assumes sibling folders match this one; excludes copied "
          "non-DNG files.")
    print("=" * 60 + "\n")


# ---------------------------------------------------------------------------
# Full dataset processing
# ---------------------------------------------------------------------------

def process_dataset(
    src_root: Path,
    dst_root: Path,
    fmt: str,
    fps: int,
    crf: int,
    preset: str,
    jpeg_quality: int,
    workers: int,
    resume: bool,
) -> None:
    """
    Walk *src_root*, convert every DNG (to one MP4 per folder, or to per-frame
    JPEGs) and copy everything else.

    DNG conversion is CPU-bound → multiprocessing.Pool.  Non-DNG copying is
    I/O-bound → ThreadPoolExecutor.
    """
    # ---- collect work -------------------------------------------------------
    # os.walk classifies files vs. dirs for us (no per-entry stat() call) and
    # lets us show progress: on a network mount, scanning the tree can itself
    # take a while.
    dng_files: list[Path] = []
    other_files: list[Path] = []
    print("Scanning source tree …")
    with tqdm(unit="dir", dynamic_ncols=True) as bar:
        for dirpath, _dirnames, filenames in os.walk(src_root):
            base = Path(dirpath)
            for name in filenames:
                if name.lower().endswith(".dng"):
                    dng_files.append(base / name)
                else:
                    other_files.append(base / name)
            bar.update(1)
            bar.set_postfix(dng=len(dng_files), other=len(other_files))

    groups = group_dng_by_folder(dng_files)

    print(f"Found {len(dng_files):,} DNG frames in {len(groups):,} folders "
          f"and {len(other_files):,} other files "
          f"({len(dng_files) + len(other_files):,} total).")
    print(f"Workers  : {workers}")

    # ---- copy non-DNG files (I/O-bound, ThreadPool) -------------------------
    copy_pairs = [(f, dst_path_for(f, src_root, dst_root)) for f in other_files]
    if resume:
        before = len(copy_pairs)
        # Skip files already copied in full (existing dst with matching size);
        # a size mismatch means a partial/interrupted copy → redo it.
        copy_pairs = [
            (s, d) for s, d in copy_pairs
            if not (d.exists() and d.stat().st_size == s.stat().st_size)
        ]
        print(f"\nCopying non-DNG files … (resume: {before - len(copy_pairs):,} "
              f"already present, {len(copy_pairs):,} to copy)")
    else:
        print("\nCopying non-DNG files …")
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(copy_non_dng, s, d): s for s, d in copy_pairs
        }
        for _ in tqdm(as_completed(futures), total=len(futures),
                      unit="file", dynamic_ncols=True):
            pass

    total_src_bytes = 0
    total_dst_bytes = 0
    total_frames = len(dng_files)
    failures: list[tuple[str, str]] = []  # (ident, error) per failed item

    if fmt == "mp4":
        # ---- encode DNG folders → MP4 (CPU-bound, ProcessPool) --------------
        # Existing .mp4 files are complete (encode is atomic), so --resume can
        # safely skip them.
        tasks = []
        skipped = 0
        for folder, frames in sorted(groups.items()):
            dst = dst_mp4_path(folder, src_root, dst_root)
            if resume and dst.exists():
                skipped += 1
                continue
            tasks.append((frames, dst, fps, crf, preset))
        msg = f" ({skipped:,} already done, {len(tasks):,} to encode)" if resume else ""
        print(f"\nEncoding {len(groups):,} folders to MP4 "
              f"(libx264 crf={crf} preset={preset} fps={fps}){msg} …")
        # maxtasksperchild=1: restart each worker after every folder so libraw's
        # per-decode memory growth is reclaimed by the OS instead of piling up
        # over the whole run.  One folder = thousands of frames, so the restart
        # cost is negligible.
        with Pool(processes=workers, maxtasksperchild=1) as pool:
            results = pool.imap_unordered(_encode_worker, tasks, chunksize=1)
            for src_b, dst_b, _n, err, ident in tqdm(
                    results, total=len(tasks), unit="folder", dynamic_ncols=True):
                if err:
                    failures.append((ident, err))
                else:
                    total_src_bytes += src_b
                    total_dst_bytes += dst_b
    else:
        # ---- convert DNG → JPEG (CPU-bound, ProcessPool) -------------------
        # Existing .jpg files are complete (write is atomic) → safe to skip.
        tasks = []
        skipped = 0
        for dng in dng_files:
            dst = dst_jpeg_path(dng, src_root, dst_root)
            if resume and dst.exists():
                skipped += 1
                continue
            tasks.append((dng, dst, jpeg_quality))
        msg = f" ({skipped:,} already done, {len(tasks):,} to convert)" if resume else ""
        print(f"\nConverting {len(dng_files):,} DNG files to JPEG "
              f"(quality={jpeg_quality}){msg} …")
        # Recycle workers periodically (each task is just one frame here) to cap
        # libraw's cumulative per-decode memory growth.
        with Pool(processes=workers, maxtasksperchild=16) as pool:
            results = pool.imap_unordered(_jpeg_worker, tasks, chunksize=4)
            for src_b, dst_b, _n, err, ident in tqdm(
                    results, total=len(tasks), unit="file", dynamic_ncols=True):
                if err:
                    failures.append((ident, err))
                else:
                    total_src_bytes += src_b
                    total_dst_bytes += dst_b

    # ---- summary ------------------------------------------------------------
    ratio = total_src_bytes / total_dst_bytes if total_dst_bytes else float("inf")
    out_label = "MP4 total " if fmt == "mp4" else "JPEG total"
    print("\n" + "=" * 60)
    print("  DONE")
    print("=" * 60)
    print(f"  Folders    : {len(groups):,}  ({total_frames:,} frames)")
    print(f"  DNG total  : {fmt_size(total_src_bytes)}")
    print(f"  {out_label} : {fmt_size(total_dst_bytes)}")
    print(f"  Ratio      : {ratio:.1f}×  (saved {100 - 100/ratio:.1f}%)")
    print(f"  Workers    : {workers}")
    print(f"  Destination: {dst_root}")
    print(f"  Failed     : {len(failures):,}")
    print("=" * 60)

    if failures:
        shown = failures[:20]
        print(f"\n⚠  {len(failures):,} item(s) failed (e.g. transient mount I/O "
              f"errors). Showing first {len(shown)}:")
        for ident, err in shown:
            print(f"   - {ident}\n       {err}")
        if len(failures) > len(shown):
            print(f"   … and {len(failures) - len(shown):,} more.")
        print("\nRe-run the same command with --resume to retry only the "
              "failed/unfinished items.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compress a multi-camera DNG mocap dataset to H.264 MP4.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--src",
        required=True,
        metavar="PATH",
        help="Root of the source dataset (e.g. …/Violin_test_Apr26/Source).",
    )
    parser.add_argument(
        "--dst",
        required=True,
        metavar="PATH",
        help="Root of the output (compressed) dataset.",
    )
    parser.add_argument(
        "--format",
        choices=("mp4", "jpg"),
        default="mp4",
        help="Output format: one H.264 MP4 per folder, or one JPEG per DNG.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=90,
        metavar="1-95",
        help="JPEG quality when --format jpg (1 = smallest, 95 = best).",
    )
    parser.add_argument(
        "--crf",
        type=int,
        default=30,
        metavar="0-51",
        help="x264 CRF for --format mp4 (0 = lossless, 51 = worst).",
    )
    parser.add_argument(
        "--lossless",
        action="store_true",
        help="Lossless encoding (forces --crf 0). Note: with yuv420p chroma is "
             "still subsampled, so it is mathematically lossless only in luma.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=50,
        metavar="N",
        help="Output frame rate (capture rate of the cameras).",
    )
    parser.add_argument(
        "--preset",
        default="fast",
        metavar="NAME",
        help="x264 preset (ultrafast … placebo).",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=os.cpu_count(),
        metavar="N",
        help="Number of parallel workers (default: all CPU cores).",
    )
    parser.add_argument(
        "--preview-only",
        action="store_true",
        help="Encode one folder as a preview, then exit without processing "
             "the full dataset.",
    )
    parser.add_argument(
        "--skip-preview",
        action="store_true",
        help="Skip the preview step and go straight to full processing.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip outputs that already exist in --dst (completed MP4s/JPEGs "
             "and fully-copied files), so an interrupted run continues instead "
             "of starting over.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.lossless:
        args.crf = 0

    if not 0 <= args.crf <= 51:
        print("Error: --crf must be between 0 and 51.", file=sys.stderr)
        sys.exit(1)

    if not 1 <= args.jpeg_quality <= 95:
        print("Error: --jpeg-quality must be between 1 and 95.", file=sys.stderr)
        sys.exit(1)

    if args.fps < 1:
        print("Error: --fps must be at least 1.", file=sys.stderr)
        sys.exit(1)

    if args.workers < 1:
        print("Error: --workers must be at least 1.", file=sys.stderr)
        sys.exit(1)

    src_root = Path(args.src)
    dst_root = Path(args.dst)

    if not src_root.exists():
        print(f"Error: source path does not exist: {src_root}", file=sys.stderr)
        sys.exit(1)

    if args.format == "mp4":
        ensure_ffmpeg()
        encode_desc = f"libx264 crf={args.crf} preset={args.preset} fps={args.fps}"
    else:
        encode_desc = f"JPEG quality={args.jpeg_quality}"

    print(f"Source : {src_root}")
    print(f"Output : {dst_root}")
    print(f"Format : {args.format}  ({encode_desc})")

    # ---- preview ------------------------------------------------------------
    if not args.skip_preview:
        run_preview(src_root, dst_root, args.format, args.fps, args.crf,
                    args.preset, args.jpeg_quality)

        if args.preview_only:
            print("--preview-only flag set.  Exiting without full processing.")
            return

        answer = input("Proceed with full dataset compression? [y/N] ").strip().lower()
        if answer != "y":
            print("Aborted.")
            return

    # ---- full run -----------------------------------------------------------
    process_dataset(src_root, dst_root, args.format, args.fps, args.crf,
                    args.preset, args.jpeg_quality, args.workers, args.resume)


if __name__ == "__main__":
    # rawpy uses OpenMP internally; 'fork' can deadlock when OpenMP threads are
    # active.  'forkserver' starts a clean server process to spawn workers from,
    # avoiding the issue entirely.
    mp.set_start_method("forkserver")
    main()
