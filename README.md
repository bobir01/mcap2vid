# mcap2vid

[![Crates.io](https://img.shields.io/crates/v/mcap2vid)](https://crates.io/crates/mcap2vid)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

High-performance MCAP to MP4 video extractor with embedded ROS2 timestamp preservation.  
Written in Rust — blazing fast, memory-efficient, S3-native.

```bash
curl -sSfL https://bobir01.github.io/mcap2vid/install.sh | sh
```

---

## Features

- 🎥 **Extract video** from MCAP recordings to H.264 MP4
- 🕐 **Preserve ROS2 timestamps** — nanosecond-precision via custom `FTSS` MP4 atom
- ⚡ **Parallel frame decoding** powered by [rayon](https://github.com/rayon-rs/rayon)
- 🗂 **Memory-mapped MCAP reading** — handles files of any size with low RAM footprint
- ☁️ **S3-compatible remote reading** — stream directly from S3/MinIO/Ceph without downloading the full file (selective chunk fetching)
- 📦 **Compressed & raw image formats** — JPEG, PNG, rgb8, bgr8, rgba8, bgra8, mono8, mono16
- 📐 **Camera calibration export** — dump `CameraInfo` to JSON
- 🔄 **Transform export** — dump TF/TF_static to JSON with optional frame filtering
- 📡 **Stdout streaming** — pipe directly to FFmpeg/other tools with zero disk writes

---

## Requirements

- [FFmpeg](https://ffmpeg.org/download.html) must be installed and available in `PATH`

---

## Installation

**Quick install** (Linux x86\_64 / aarch64):

```bash
curl -sSfL https://bobir01.github.io/mcap2vid/install.sh | sh
```

**Via Cargo:**

```bash
cargo install mcap2vid
```

Pre-built binaries are also available on the [GitHub Releases](https://github.com/bobir01/mcap2vid/releases) page.

---

## Usage

### List video topics

```bash
# From a local file
mcap2vid list -i recording.mcap

# From S3-compatible storage
mcap2vid list -i s3c://my-bucket/path/to/recording.mcap
```

Prints a table of available image topics, their schema type, and message count.

---

### Export video

```bash
# Basic export
mcap2vid export -i recording.mcap -t /camera/image_raw -o output.mp4

# Custom encoding settings
mcap2vid export -i recording.mcap -t /camera/image_raw -o output.mp4 \
  --preset fast --crf 23 --threads 8

# Add a suffix to the output filename (produces output_ego.mp4)
mcap2vid export -i recording.mcap -t /camera/image_raw -o output.mp4 --suffix ego

# Stream to stdout (zero disk writes — useful for piping)
mcap2vid export -i recording.mcap -t /camera/image_raw -o - | ffplay -i pipe:0

# Export directly from S3
mcap2vid export -i s3c://my-bucket/path/recording.mcap -t /camera/image_raw -o output.mp4
```

**Export options:**

| Flag | Default | Description |
|------|---------|-------------|
| `-i, --input` | _(required)_ | Local file path or `s3c://` URL |
| `-t, --topic` | _(required)_ | ROS2 image topic (e.g. `/camera/image_raw`) |
| `-o, --output` | _(required)_ | Output `.mp4` path, or `-` for stdout |
| `--suffix` | _(none)_ | Appended to output filename (e.g. `ego` → `name_ego.mp4`) |
| `--preset` | `medium` | FFmpeg encoding preset (`ultrafast`, `fast`, `medium`, `slow`) |
| `--crf` | `18` | Constant Rate Factor — quality (0–51, lower = better) |
| `--threads` | `0` (auto) | Parallel decode threads (0 = auto-detect) |

> **Note:** Timestamps are embedded into the output MP4 in a custom `FTSS` atom. They are **not** embedded when streaming to stdout (`-o -`).

---

### Verify timestamps

```bash
# Show first/last 5 timestamps
mcap2vid verify -i output.mp4

# Show all timestamps
mcap2vid verify -i output.mp4 --all
```

Reads the embedded `FTSS` atom and prints a table of frame index, Unix timestamp, and inter-frame delta in milliseconds.

---

### Export metadata

#### List metadata topics

```bash
mcap2vid metadata list -i recording.mcap
mcap2vid metadata list -i s3c://my-bucket/recording.mcap
```

#### Export CameraInfo

```bash
# Export all CameraInfo messages to JSON
mcap2vid metadata camera-info -i recording.mcap -t /camera/camera_info -o camera.json --pretty

# Export only the first message (useful when all messages are identical)
mcap2vid metadata camera-info -i recording.mcap -t /camera/camera_info --first-only

# Print to stdout
mcap2vid metadata camera-info -i recording.mcap -t /camera/camera_info
```

**CameraInfo options:**

| Flag | Default | Description |
|------|---------|-------------|
| `-i, --input` | _(required)_ | Local file path or `s3c://` URL |
| `-t, --topic` | _(required)_ | CameraInfo topic |
| `-o, --output` | _(stdout)_ | Output JSON file path |
| `--first-only` | false | Export only the first message |
| `--pretty` | false | Pretty-print JSON |

#### Export TF transforms

```bash
# Export all transforms from /tf_static
mcap2vid metadata tf -i recording.mcap -t /tf_static -o transforms.json --pretty

# Filter by parent frame
mcap2vid metadata tf -i recording.mcap -t /tf --parent-frame base_link

# Filter by child frame
mcap2vid metadata tf -i recording.mcap -t /tf --child-frame camera_optical

# Both filters combined
mcap2vid metadata tf -i recording.mcap -t /tf \
  --parent-frame base_link --child-frame camera_optical -o filtered.json --pretty
```

**TF options:**

| Flag | Default | Description |
|------|---------|-------------|
| `-i, --input` | _(required)_ | Local file path or `s3c://` URL |
| `-t, --topic` | `/tf` | TF topic to extract |
| `-o, --output` | _(stdout)_ | Output JSON file path |
| `--parent-frame` | _(none)_ | Filter transforms by parent `frame_id` |
| `--child-frame` | _(none)_ | Filter transforms by `child_frame_id` |
| `--pretty` | false | Pretty-print JSON |

---

## S3-Compatible Storage

`mcap2vid` can read MCAP files directly from any S3-compatible object store (AWS S3, MinIO, Ceph, Hyperstack, etc.) using the `s3c://` URL scheme.

**URL format:**
```
s3c://bucket-name/path/to/recording.mcap
```

**Required environment variables:**

| Variable | Description |
|----------|-------------|
| `S3_ENDPOINT` | Full endpoint URL (e.g. `https://s3.amazonaws.com`) |
| `S3_ACCESS_KEY` | Access key ID |
| `S3_SECRET_KEY` | Secret access key |

**Example:**

```bash
export S3_ENDPOINT=https://ca1.obj.nexgencloud.io
export S3_ACCESS_KEY=your-access-key
export S3_SECRET_KEY=your-secret-key

mcap2vid list -i s3c://my-bucket/recordings/2024-01-01.mcap
mcap2vid export -i s3c://my-bucket/recordings/2024-01-01.mcap \
  -t /camera/image_raw -o output.mp4
```

**How it works:**  
S3 reads are highly optimized — `mcap2vid` fetches only the MCAP summary index (a few KB from the end of the file) to discover topics and chunk offsets, then downloads only the chunks relevant to the requested topic. This avoids downloading entire multi-GB recordings when you only need one camera stream.

---

## How Timestamps Work

ROS2 MCAP recordings carry nanosecond-precision timestamps on every message. Standard MP4 files lose this precision.

`mcap2vid` solves this by embedding all frame timestamps into a custom `FTSS` (Frame Timestamp Store) MP4 atom after encoding. The atom stores a compact array of 64-bit floats (Unix epoch, seconds) — one per frame.

Use `mcap2vid verify` to read them back out at any time.

---

## License

MIT

---

*Built for robotics engineers working with ROS2 / autonomous vehicle data pipelines.*
