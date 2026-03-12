# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build (development)
cargo build

# Build (release, optimized with LTO)
cargo build --release

# Run directly
cargo run -- <subcommand>

# Run release binary
./target/release/mcap2vid <subcommand>
```

## Usage

```bash
# List video topics in an MCAP file
mcap2vid list -i recording.mcap

# Export video from MCAP to MP4
mcap2vid export -i recording.mcap -t /camera/image_raw -o output.mp4

# Export with custom encoding settings
mcap2vid export -i recording.mcap -t /camera/image_raw -o output.mp4 --preset fast --crf 23 --threads 8

# Verify embedded timestamps in MP4
mcap2vid verify -i output.mp4
mcap2vid verify -i output.mp4 --all  # Show all timestamps

# List metadata topics (CameraInfo, TF)
mcap2vid metadata list -i recording.mcap

# Export camera intrinsics to JSON
mcap2vid metadata camera-info -i recording.mcap -t /camera/camera_info -o camera.json --pretty
mcap2vid metadata camera-info -i recording.mcap -t /camera/camera_info --first-only

# Export TF transforms to JSON
mcap2vid metadata tf -i recording.mcap -t /tf_static -o transforms.json --pretty
mcap2vid metadata tf -i recording.mcap --parent-frame world --child-frame camera_link

# --- S3-compatible remote access (s3c://) ---
# Requires: S3_ENDPOINT, S3_ACCESS_KEY, S3_SECRET_KEY env vars

# List topics from remote MCAP
mcap2vid list -i s3c://bucket/path/to/recording.mcap

# Export video from S3 to stdout (zero disk writes)
mcap2vid export -i s3c://bucket/path/to/recording.mcap -t /camera/image_raw -o -

# Export from S3 to local file
mcap2vid export -i s3c://bucket/path/to/recording.mcap -t /camera/image_raw -o output.mp4

# Pipe S3 export to ffplay for live preview
mcap2vid export -i s3c://bucket/recording.mcap -t /camera/image_raw -o - 2>/dev/null | ffplay -
```

## Architecture

This is a high-performance MCAP-to-MP4 video extractor that preserves original ROS2 timestamps.

**Data Flow:**
1. `McapReader` - Memory-maps MCAP file (or loads from S3 via `s3_reader`), parses ROS2 CDR-encoded messages, extracts `VideoFrame`s
2. `decoder` - Parallel decoding (via rayon) of compressed (JPEG/PNG) or raw image data to RGB24 `DecodedFrame`s
3. `FfmpegEncoder` - Pipes raw RGB24 frames to FFmpeg subprocess for H.264 encoding
4. `timestamp` - Embeds original ROS2 timestamps as custom FTSS atom appended to MP4

**Key Design Decisions:**
- Uses memory-mapped file access for efficient local MCAP reading
- S3-compatible remote access via `s3c://` URLs with AWS SigV4 auth (no tokio/async runtime needed)
- Parallel frame decoding with rayon
- Pipes to FFmpeg via stdin rather than writing intermediate files
- Custom FTSS (Frame TimeStamp Sync) MP4 atom stores nanosecond-precision Unix timestamps without modifying moov structure
- Stdout output (`-o -`) uses fragmented MP4 for streaming without disk writes

**Supported Image Formats:**
- Compressed: JPEG, PNG (via `sensor_msgs/CompressedImage`)
- Raw encodings: rgb8, bgr8, rgba8, bgra8, mono8, mono16/16UC1 (via `sensor_msgs/Image`)

**Supported Metadata Formats:**
- Camera Intrinsics: `sensor_msgs/CameraInfo` (calibration matrices K, R, P, distortion coefficients)
- Transforms: `tf2_msgs/TFMessage` (static and dynamic coordinate frame transforms)

## Dependencies

- FFmpeg must be installed and available in PATH for encoding
