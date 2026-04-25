# Foxglove Protobuf Support Plan

## Goal
Add support for protobuf-based MCAP topics using Foxglove schemas, starting with:
- `foxglove.CompressedImage`
- `foxglove.CompressedVideo`

This should work for:
- local MCAP files
- `s3c://` remote reads
- topic listing
- export to MP4 where feasible

## Current state
The codebase is currently ROS2/CDR-oriented:
- `src/ros2_msgs.rs` parses CDR `sensor_msgs/Image`, `sensor_msgs/CompressedImage`, `CameraInfo`, and `TFMessage`
- `src/mcap_reader.rs` detects message types by schema name substring matching
- `src/s3_reader.rs` uses the same schema-name logic for selective remote extraction
- `src/decoder.rs` already knows how to decode compressed image payloads once bytes are extracted
- `src/encoder.rs` is RGB-frame -> H.264 MP4 via FFmpeg stdin

## Constraints / facts
- Foxglove protobuf uses different wire encoding than ROS2 CDR
- `foxglove.CompressedImage` is easy to map into the existing compressed-image pipeline
- `foxglove.CompressedVideo` is not image bytes; it is already compressed video frame data
- Foxglove `CompressedVideo` expects one decodable frame per message
- For h264/h265, payloads should be Annex B style
- B-frames are not expected/supported by Foxglove format assumptions

## Recommended implementation order
1. Add schema-aware topic detection infrastructure
2. Add protobuf support for `foxglove.CompressedImage`
3. Add protobuf support for `foxglove.CompressedVideo`
4. Add tests and sample fixtures
5. Update docs and release notes

---

## Phase 1 — schema-aware detection

### Problem
Current detection is too fuzzy:
- `ImageMessageType::from_schema_name()` matches on string containment
- it does not consider schema encoding (protobuf vs CDR)

### Change
Introduce a richer message kind model.

Suggested shape:
- `SchemaEncodingKind`:
  - `Ros2Cdr`
  - `Protobuf`
  - `Unknown`
- `VideoMessageKind`:
  - `Ros2Image`
  - `Ros2CompressedImage`
  - `FoxgloveCompressedImageProto`
  - `FoxgloveCompressedVideoProto`

### Touch points
- `src/mcap_reader.rs`
- `src/s3_reader.rs`
- likely move schema classification into a dedicated module, e.g. `src/schema.rs`

### Outcome
All listing/extraction code branches on exact message kind rather than substring heuristics.

---

## Phase 2 — protobuf `foxglove.CompressedImage`

### Goal
Support MCAP channels with schema name:
- `foxglove.CompressedImage`

### Data mapping
Map protobuf fields to existing `VideoFrame` model:
- `timestamp` -> `VideoFrame.timestamp`
- `data` -> `FrameData::Compressed.data`
- `format` -> `FrameData::Compressed.format`

### Implementation approach
Use `prost` + `prost-build`.

Suggested additions:
- `build.rs`
- `proto/foxglove/CompressedImage.proto`
- `proto/foxglove/CompressedVideo.proto`
- generated module wrapper, e.g. `src/foxglove_proto.rs`

### Cargo additions
Likely:
- `prost`
- `prost-types`
- build-dependency `prost-build`

### Code changes
- Add protobuf decode helper for `foxglove.CompressedImage`
- Extend local extraction in `src/mcap_reader.rs`
- Extend remote extraction in `src/s3_reader.rs`
- Ensure topic listing surfaces these channels as video topics

### Expected difficulty
Low to medium.
This should reuse the current compressed-image decode path in `src/decoder.rs` almost directly.

---

## Phase 3 — protobuf `foxglove.CompressedVideo`

### Goal
Support MCAP channels with schema name:
- `foxglove.CompressedVideo`

### Important design choice
This should NOT be forced through the current image-decoding path.

Current pipeline:
1. extract bytes
2. decode to RGB images
3. pipe raw RGB into ffmpeg
4. encode to MP4

For `CompressedVideo`, the payload is already video bitstream data.

### Recommended architecture
Add a second export path for packetized compressed video.

Suggested internal model extension:
- `FrameData::CompressedVideoPacket { format, data }`

Then add export logic that:
- writes packet payloads into an FFmpeg subprocess configured to ingest the compressed elementary stream
- remuxes and/or transcodes to MP4
- still preserves per-message timestamps for FTSS embedding if possible

### Notes
- h264/h265: likely `ffmpeg -f h264 -i -` / analogous handling, but exact framing needs testing with sample MCAPs
- vp9/av1: may require different demux assumptions and should be validated with real samples
- if timestamp-preserving remux proves messy, first goal should be “successful MP4 export”; FTSS support can follow if needed

### Expected difficulty
Medium to high.
This part needs real sample files before implementation is safe.

---

## Phase 4 — tests / fixtures

### Need from Bob later
Sample protobuf-based MCAP files for:
- `foxglove.CompressedImage`
- `foxglove.CompressedVideo`
- ideally one S3-hosted sample too

### Test plan
- unit tests for schema classification
- unit tests for protobuf decode helpers
- integration tests for:
  - `list`
  - `export`
  - `verify`
- one remote selective-read test for `s3c://` path if practical

### Edge cases to test
- unknown/unsupported `format`
- empty payload
- malformed protobuf message
- `CompressedVideo` keyframe requirements not met
- mixed schemas in one MCAP

---

## Phase 5 — docs / release

### Update docs
- `README.md`
- `docs/index.html`
- release notes/tag notes

### Document exact support matrix
- ROS2 CDR `sensor_msgs/Image`
- ROS2 CDR `sensor_msgs/CompressedImage`
- Foxglove protobuf `CompressedImage`
- Foxglove protobuf `CompressedVideo` (once implemented)

### Mention format support explicitly
- `CompressedImage`: `jpeg`, `png`, maybe `webp`/`avif` depending on `image` crate support in practice
- `CompressedVideo`: `h264`, `h265`, `vp9`, `av1` only after validation with real samples

---

## Suggested first coding slice
When sample files arrive, start with this small vertical slice:
1. add exact schema classification
2. add `prost` codegen for `foxglove.CompressedImage`
3. make `list` show protobuf compressed image topics
4. make `export` work for protobuf compressed image topics
5. test locally and from `s3c://`

Only after that, start `foxglove.CompressedVideo`.

## Nice-to-have follow-ups
- add `mcap2vid inspect-schema` or verbose listing mode showing schema encoding + exact schema name
- add changelog file (`CHANGELOG.md`) since this feature is substantial
- add CI matrix test fixtures once sample files exist
