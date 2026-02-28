# mcap2vid

High-performance MCAP to MP4 video extractor with embedded ROS2 timestamp preservation.

```bash
curl -sSfL https://bobir01.github.io/mcap2vid/install.sh | sh
```

## Features

- Extract video from MCAP recordings to MP4
- Preserve original ROS2 nanosecond-precision timestamps via custom FTSS atom
- Parallel frame decoding with rayon
- Memory-mapped MCAP reading for efficiency
- Support for compressed (JPEG, PNG) and raw image formats (rgb8, bgr8, rgba8, bgra8, mono8, mono16)
- Export camera calibration (CameraInfo) and transforms (TF) as JSON

## Requirements

- FFmpeg must be installed and available in PATH

## Installation

**Quick install** (Linux x86_64/aarch64):

```bash
curl -sSfL https://bobir01.github.io/mcap2vid/install.sh | sh
```

**Via Cargo:**

```bash
cargo install mcap2vid
```

Pre-built binaries are also available on the [GitHub Releases](https://github.com/bobir01/mcap2vid/releases) page.

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
mcap2vid verify -i output.mp4 --all

# Export metadata
mcap2vid metadata list -i recording.mcap
mcap2vid metadata camera-info -i recording.mcap -t /camera/camera_info -o camera.json --pretty
mcap2vid metadata tf -i recording.mcap -t /tf_static -o transforms.json --pretty
```

## License

MIT
